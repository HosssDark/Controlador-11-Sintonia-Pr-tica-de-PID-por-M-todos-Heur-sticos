import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time
import sys

#  CONFIGURACAO
PORTA_SERIAL = "COM3"        # AJUSTE AQUI para sua porta
BAUD_RATE = 115200
JANELA_AMOSTRAS = 300         # quantos pontos mostrar no grafico (300 * 10ms = 3s)
TS_ESPERADO_MS = 10           # tempo de amostragem do firmware

#  DETECCAO AUTOMATICA DE PORTA (auxiliar)
def listar_portas_disponiveis():
    portas = serial.tools.list_ports.comports()
    if not portas:
        print("Nenhuma porta serial encontrada.")
        return []
    print("\nPortas seriais disponiveis:")
    for p in portas:
        print(f"  {p.device}  -  {p.description}")
    return [p.device for p in portas]


#  BUFFERS DE DADOS (thread-safe via deque)
tempo_buf = deque(maxlen=JANELA_AMOSTRAS)
setpoint_buf = deque(maxlen=JANELA_AMOSTRAS)
posicao_buf = deque(maxlen=JANELA_AMOSTRAS)
erro_buf = deque(maxlen=JANELA_AMOSTRAS)
pwm_buf = deque(maxlen=JANELA_AMOSTRAS)

lock = threading.Lock()
contador_amostras = 0
conexao_ok = False
ultimo_status = ""


#  THREAD DE LEITURA SERIAL
def thread_leitura_serial(ser):
    global contador_amostras, conexao_ok, ultimo_status

    while True:
        try:
            linha = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception as e:
            print(f"\n[ERRO] Falha na leitura serial: {e}")
            conexao_ok = False
            return

        if not linha:
            continue

        # Linhas de comentario/status do firmware comecam com '#'
        if linha.startswith("#"):
            ultimo_status = linha
            print(f"[ARDUINO] {linha}")
            continue

        # Ignora o cabecalho CSV
        if linha.startswith("Setpoint"):
            conexao_ok = True
            continue

        # Tenta parsear a linha CSV: Setpoint,Posicao,Erro,PWM
        partes = linha.split(",")
        if len(partes) < 4:
            continue

        try:
            sp = float(partes[0])
            pos = float(partes[1])
            err = float(partes[2])
            pwm = float(partes[3])
        except ValueError:
            continue

        with lock:
            conexao_ok = True
            contador_amostras += 1
            tempo_s = contador_amostras * (TS_ESPERADO_MS / 1000.0)
            tempo_buf.append(tempo_s)
            setpoint_buf.append(sp)
            posicao_buf.append(pos)
            erro_buf.append(err)
            pwm_buf.append(pwm)


#  THREAD DE ENTRADA DE COMANDOS (terminal -> Arduino)
def thread_comandos(ser):
    print("\nDigite comandos para enviar ao Arduino (ou 'sair' para encerrar):")
    print("  Exemplos: 90 | P2.5 | I4.0 | D0.08 | DIST | R | STATUS\n")
    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd.lower() == "sair":
            print("Encerrando...")
            ser.close()
            sys.exit(0)

        if cmd:
            ser.write((cmd + "\n").encode("utf-8"))


#  CONFIGURACAO DOS GRAFICOS (4 subplots)
def configurar_graficos():
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    fig.suptitle("Supervisor HMI - Controle PID da Junta Robotica", fontsize=13)

    linha_sp, = axs[0].plot([], [], label="Setpoint", color="#185FA5", linewidth=1.5)
    linha_pos, = axs[0].plot([], [], label="Posicao", color="#D85A30", linewidth=1.5)
    axs[0].set_ylabel("Angulo (graus)")
    axs[0].legend(loc="upper right", fontsize=9)
    axs[0].grid(True, alpha=0.3)

    linha_erro, = axs[1].plot([], [], color="#993C1D", linewidth=1.2)
    axs[1].axhline(0, color="gray", linewidth=0.5, linestyle="--")
    axs[1].set_ylabel("Erro (graus)")
    axs[1].grid(True, alpha=0.3)

    linha_pwm, = axs[2].plot([], [], color="#1D9E75", linewidth=1.2)
    axs[2].axhline(0, color="gray", linewidth=0.5, linestyle="--")
    axs[2].axhline(200, color="red", linewidth=0.5, linestyle=":")
    axs[2].axhline(-200, color="red", linewidth=0.5, linestyle=":")
    axs[2].set_ylabel("PWM (saturacao)")
    axs[2].grid(True, alpha=0.3)

    # Painel de status textual
    axs[3].axis("off")
    texto_status = axs[3].text(
        0.0, 0.5, "", fontsize=11, family="monospace",
        verticalalignment="center", transform=axs[3].transAxes
    )

    axs[2].set_xlabel("Tempo (s)")

    plt.tight_layout()

    return fig, axs, linha_sp, linha_pos, linha_erro, linha_pwm, texto_status


#  FUNCAO DE ATUALIZACAO DA ANIMACAO
def atualizar(frame, axs, linha_sp, linha_pos, linha_erro, linha_pwm, texto_status):
    with lock:
        t = list(tempo_buf)
        sp = list(setpoint_buf)
        pos = list(posicao_buf)
        err = list(erro_buf)
        pwm = list(pwm_buf)

    if len(t) < 2:
        return linha_sp, linha_pos, linha_erro, linha_pwm, texto_status

    linha_sp.set_data(t, sp)
    linha_pos.set_data(t, pos)
    linha_erro.set_data(t, err)
    linha_pwm.set_data(t, pwm)

    for ax in axs[:3]:
        ax.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1)

    axs[0].relim(); axs[0].autoscale_view(scalex=False)
    axs[1].relim(); axs[1].autoscale_view(scalex=False)
    axs[2].set_ylim(-220, 220)

    # Painel de status
    sp_atual = sp[-1] if sp else 0
    pos_atual = pos[-1] if pos else 0
    err_atual = err[-1] if err else 0
    pwm_atual = pwm[-1] if pwm else 0

    status_txt = (
        f"Conexao: {'OK' if conexao_ok else 'SEM DADOS'}\n"
        f"Setpoint atual: {sp_atual:6.1f} graus\n"
        f"Posicao atual:  {pos_atual:6.1f} graus\n"
        f"Erro atual:     {err_atual:6.1f} graus\n"
        f"PWM atual:      {pwm_atual:6.0f}\n"
        f"Amostras recebidas: {contador_amostras}\n"
        f"Ultimo status Arduino: {ultimo_status}"
    )
    texto_status.set_text(status_txt)

    return linha_sp, linha_pos, linha_erro, linha_pwm, texto_status


#  PROGRAMA PRINCIPAL
def main():
    global PORTA_SERIAL

    print("=" * 60)
    print(" SUPERVISOR HMI - CONTROLE PID DE JUNTA ROBOTICA")
    print("=" * 60)

    portas = listar_portas_disponiveis()

    porta_escolhida = PORTA_SERIAL
    if portas and PORTA_SERIAL not in portas:
        print(f"\nAviso: porta configurada '{PORTA_SERIAL}' nao encontrada na lista.")
        resposta = input(f"Usar mesmo assim '{PORTA_SERIAL}'? (s/n): ").strip().lower()
        if resposta != "s":
            if len(portas) == 1:
                porta_escolhida = portas[0]
                print(f"Usando a unica porta disponivel: {porta_escolhida}")
            else:
                porta_escolhida = input("Digite a porta correta (ex: COM3): ").strip()

    try:
        ser = serial.Serial(porta_escolhida, BAUD_RATE, timeout=1)
        time.sleep(2)  # tempo para o Arduino resetar e iniciar
        print(f"\nConectado em {porta_escolhida} a {BAUD_RATE} baud.")
    except Exception as e:
        print(f"\n[ERRO] Nao foi possivel abrir a porta serial: {e}")
        print("Verifique se o Arduino esta conectado e a porta esta correta.")
        sys.exit(1)

    # Thread de leitura serial (background)
    t_leitura = threading.Thread(target=thread_leitura_serial, args=(ser,), daemon=True)
    t_leitura.start()

    # Thread de comandos via terminal (background)
    t_cmd = threading.Thread(target=thread_comandos, args=(ser,), daemon=True)
    t_cmd.start()

    # Configura e roda a animacao matplotlib (thread principal)
    fig, axs, linha_sp, linha_pos, linha_erro, linha_pwm, texto_status = configurar_graficos()

    ani = animation.FuncAnimation(
        fig, atualizar,
        fargs=(axs, linha_sp, linha_pos, linha_erro, linha_pwm, texto_status),
        interval=100,  # atualiza o grafico a cada 100ms
        cache_frame_data=False
    )

    plt.show()

    ser.close()


if __name__ == "__main__":
    main()
