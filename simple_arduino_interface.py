import serial
import time

# Function to establish a connection with the Arduino
def connect_to_arduino(port, baud_rate=9600):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2) # wait for the serial connection to initialize
        print(f"Connected to Arduino on {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to Arduino on {port}: {e}")
        return None

# Function to send a command to the Arduino
def send_command(ser, command):
    ser.write(command.encode())
    time.sleep(0.1) # wait for the Arduino to process the command
    while ser.in_waiting:
        print(ser.readline().decode().strip())

# Main CLI loop
def main():
    port = input("Enter the COM port for Arduino (e.g., COM3 or 
/dev/ttyACM0): ")
    baud_rate = input("Enter the baud rate (default 9600): ")
    baud_rate = int(baud_rate) if baud_rate else 9600
    ser = connect_to_arduino(port, baud_rate)

    if ser is not None:
        try:
            while True:
                command = input("Enter command to send or 'exit' to quit: 
")
                if command.lower() == 'exit':
                    break
                send_command(ser, command + '\n') # Arduino commands are 
typically terminated with a newline
        except KeyboardInterrupt:
            print("\nExiting program")
        finally:
            ser.close()
            print("Serial connection closed.")
    else:
        print("Unable to establish connection with Arduino. Exiting.")

if __name__ == "__main__":
    main()

