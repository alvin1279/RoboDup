import websocket
import time


# Function to send the command via WebSocket
def send_command(command):
    # Connect to the WebSocket server on the bot
    ws = websocket.WebSocket()
    bot_ip = "ws://192.168.83.196:81"  # Replace with your bot's IP address
    ws.connect(bot_ip)

    # Send the command to the bot
    print(f"Sending command: {command}")
    ws.send(command)

    # Close the WebSocket connection
    ws.close()


# Main function to take user input and send commands
def main():
    print("Enter the commands in the format: direction duration, direction duration, ...")
    print("Example: forward 1000, right 500, fast_left 200, fast_forward 1000")

    while True:
        # Get user input for the command sequence
        user_input = input("Enter the command: ")

        # Send the command to the bot
        send_command(user_input)

        # Wait for 2 seconds before accepting the next input
        # time.sleep(2)


if __name__ == "__main__":
    main()
