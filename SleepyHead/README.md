# setup VS Code for Arduino
https://forum.allaboutcircuits.com/threads/how-to-setup-and-use-vs-code-windows-for-arduino-programming.193422/
****Pick the extension that is a circle with Arduino in middle called 'Arduino Community Edition'
  plug in a board and restart vscode
  have to 'Select Programmer' in bottom bar - nano is a SAM board
  I think you have to use Arduino IDE to install libraries and VS Code automatically finds them
  to get squiggles working correctly manually rebuild your IntelliSense configuration by running "Ctrl+Alt+I"


Head nods past piano threshold in any direction a little, set just buzzer.  Past forte threshold set buzzer and speaker tone.

Eye voltage drops when lids closed.  Eye voltage rises when pull sensor away from lids / eyeballs.  Built in reset:  when voltage high let LT = ST forced.  When voltage below a threshold, allow logic to become active after a settling time.

Rolling eyes up also sets eye closed.  Same thing happens if nod head down and keep eyes up.  That shouldn't be a problem

The buzzer has ever increasing duty cycle to go from warning to full fledged wake up.

The frequency of the buzzer is in a range that scientists call 'annoying' and sleep preventing.

Use different frequncies of tone to allow user to know whether eye or head set it.

User can reset eye logic by pulling glasses down nose.
Eye logic needs a way for the head motion to reset it so driver can keep his hands on the wheel, leading to yaw wag logic

The head nod logic uses rapid head motion ('not quiet' indication) to reset

All the various resets wash out over time.  This makes sense because the driver settles in.

Practical way to test initialization:  set plot_num_def to 10 or 11 and capture stream.  Start serial monitor.  Clear it.  Reflash the device it (with no change except plot_num_def ) and run until satisfied then type pp99 to stop.  Ctrl-a to select all in serial monitor.  Paste into LibreOffice spreadsheet and save as a csv file in pySleepyHead/dataReduction.  Go to bottom of pySleepyHead/CompareRunSim.py and point to the file.  Run in pyCharm.

To capture stream:  start serial monitor, type pp<# of choice>.  Then type something like pp99 to stop.  Ctrl-a to select all in serial monitor...as above.
