# cornholeold
ledboard2.ino has more done to it. these currently aren't fully functional and were a work in progress.

Esp32's connect to eachother.
start sending messages to eachother about the team scores.
uses a button interrupt as a task to add or remove points on the teams depending on what buttons are pressed

4 buttons
team1 + and - score
team2 + and - score 

both esp32's are sending the team scores constantly and updating with eachother. 

fastled is used to increment the LED strips with lights depending on the score count. 
This could also be done with a 7 segment display if someone wants to, but just putting leds on the side and numbers should be fine.

These are my old sketches for the LED cornhole using the esp-now protocol.

it uses 2 esp32's to talk to eachother. 

I only was implementing the LEDS on Board 2 for testing right now, and board 1 was just buttons to send data and recieve. 

if you want to make them both light up and work, you're going to have to modify led board 1 to include the stuff from led board 2 and also setup the sender and reciever part in the mac addresses.

# YOU HAVE TO REPLACE YOU MAC ADDRESSES FOR THIS TO WORK!
