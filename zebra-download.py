from nis import match
import requests

match_key = input("Enter match key (e.g. 2022cmptx_sf1m8): ")
team_key = input("Enter team key (e.g. frc6328): ")
auth_key = ""

all_data = requests.get("https://www.thebluealliance.com/api/v3/match/" +
                        match_key + "/zebra_motionworks", headers={"X-TBA-Auth-Key": auth_key}).json()
csv = open(match_key + "_zebra.csv", "w")

# Get team data
team_data = None
for value in all_data["alliances"]["red"]:
    if value["team_key"] == team_key:
        team_data = value
for value in all_data["alliances"]["blue"]:
    if value["team_key"] == team_key:
        team_data = value
if team_data == None:
    print("Team \"" + team_key + "\" not found")

# Generate CSV
for i in range(len(all_data["times"])):
    entry_data = [all_data["times"][i], team_data["xs"][i], team_data["ys"][i]]
    csv.write(",".join([str(x) for x in entry_data]) + "\n")
