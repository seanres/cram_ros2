import serial
import time

def connect_to_arduino(port="/dev/ttyACM0", baud_rate=250000):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to Arduino on {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to Arduino on {port}: {e}")
        return None

def send_command(ser, command):
    ser.write(command.encode())
    time.sleep(0.1)
    while ser.in_waiting:
        print(ser.readline().decode().strip())

def main():
    ser = connect_to_arduino("/dev/ttyACM0", 250000)

    if ser is not None:
        try:
            while True:
                command = input("> ")
                if command.lower() == 'exit':
                    break
                send_command(ser, command.strip() + '\r\n')
        except KeyboardInterrupt:
            print("\nExiting program")
        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Unable to establish connection with Arduino. Exiting.")

if __name__ == "__main__":
    main()
