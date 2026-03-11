from ntcore import NetworkTableInstance

TEAM_NUMBER = 578

nt = NetworkTableInstance.getDefault()
nt.setServerTeam(TEAM_NUMBER)
nt.startClient4("my-client")
sd = nt.getTable("SmartDashboard")

while True:
    change = input("Change (m=Mode, c=Color, q=Quit):")
    if change == "q":
        break
    elif change == "m":
        mode = input("LED Mode: ")
        sd.putString("LED Mode", mode)
    elif change == "c":
        color = input("LED Color: ")
        sd.putString("LED Color", color)
