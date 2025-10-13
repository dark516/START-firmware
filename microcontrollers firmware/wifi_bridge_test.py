import socket

HOST = "192.168.125.241"# IP ESP32
PORT = 3333

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
print("✅ Подключено к ESP32")

try:
    while True:
        line = input("Введите v_lin v_ang (через пробел): ")
        if not line.strip():
            continue
        if line.lower() in ["q", "exit"]:
            break

        # отправляем строку с переводом строки
        sock.sendall((line.strip() + "\n").encode())
        print("➡ Отправлено:", line.strip())

finally:
    sock.close()
    print("❌ Соединение закрыто")
