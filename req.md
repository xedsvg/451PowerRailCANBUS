Power Rail control pin is D4. 

Low = Rail off.
High = Rail on.

Toggle power rail based on direct can message.
The default address is 999

ON command is 999#10...
OFF command is 999#00...

Toggle power rail based on indirect can message that we store in eeprom.
We can store exact messages or partial messages.
ex: 

ON command if we sniff 283#DEADBEEF
OFF command if we sniff 283#00000000


