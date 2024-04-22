How to control the car's action?

# What could be controlled?

- velocity
    1. Add a lapse to its node receiver, discarding all the new commands during and a short wile after performing a moving command

- arm

# What will trigger a control command?

- when obstacles detected by camera

- when target detected by camera

- distance measured by sonar

## LOGIC

- if nothing detected:<br>
    1. 

- if obstacles detected:<br>
    1. 

- if more than one target detected: <br>
    1. confirm if the two overlap with each other
    

- if one target detected:<br>
    1. adjusting car's orientation towards the target
    2. heading towards the target
    3. stop at a proper distance 
    4. reaching out arm to grab the target
    
## Modes definition

- crusing

- approaching