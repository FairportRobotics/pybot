from ntcore import NetworkTableInstance

TEAM_NUMBER = 578

nt = NetworkTableInstance.getDefault()
nt.setServerTeam(TEAM_NUMBER)
nt.startClient4("my-client")
sd = nt.getTable("SmartDashboard")

while True:
    mode = input("LED Mode: ")
    if mode == "q":
        break
    sd.putString("LED Mode", mode)
