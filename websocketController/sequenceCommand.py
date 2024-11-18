import websocket
import random
import time


# Function to send a command via an existing WebSocket connection
def send_command(ws, command):
    print(f"Sending command: {command}")
    ws.send(command)


# Main function to manage the WebSocket connection and send random commands conditionally
def main():
    # Array of possible command sequences
    commands = ['f08r01', 'b09l02', 'r02f05', 'l02b05', 'f05r02', 's05']  # Add more commands as needed

    # Connect to the WebSocket server on the bot
    bot_ip = "ws://192.168.104.196:81"  # Replace with your bot's IP address
    # jasira Ip: 192.168.104.196:81
    # hashar: 192.168.83.196:81
    ws = websocket.WebSocket()
    ws.connect(bot_ip)
    print("Connected to WebSocket server")

    initial_command = "f20r07f22b20l05b02"   # mad
    # initial_command = "f10s02f10r05s02f16b20l05f05"  # less mad
    send_command(ws, initial_command)
    time.sleep(3)
    second_command = "l10s02f08r15s02R10f10L10"
    time.sleep(4)

    try:
        # Continuously send commands based on server response
        while True:
            # Select and send a random command
            random_command = random.choice(commands)
            send_command(ws, random_command)

            # Wait for the "completed" message from the server
            while True:
                response = ws.recv()
                print(f"Received from server: {response}")
                if response == "completed":
                    print("Server confirmed completion, sending next command...")
                    time.sleep(0.5)
                    break  # Exit the inner loop to send the next command

    except KeyboardInterrupt:
        print("Terminating command loop...")

    finally:
        # Close the WebSocket connection when done
        ws.close()
        print("WebSocket connection closed")


if __name__ == "__main__":
    main()
