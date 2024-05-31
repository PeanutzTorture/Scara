class Waypoint:
    running_id = 0
    def __init__(self, angles=None, height=None, grip=None, file_data=None):
        if not file_data:
            self.angles = tuple(angles)
            self.height = height
            self.id = Waypoint.running_id
            self.grip = grip
            Waypoint.running_id += 1
        else:
            self.angles = tuple(file_data["ANGLES"])
            self.height = file_data["HEIGHT"]
            self.id = file_data["ID"]
            self.grip = file_data["GRIP"]
        
    def write(self, open_file):
        identity = "ID:" + str(self.id) + '\n'
        angles = "ANGLES:" + str(int(self.angles[0])) + ';' + str(int(self.angles[1])) + ';' + str(int(self.angles[2])) + '\n'
        height = "HEIGHT:" + str(int(self.height)) + '\n'
        grip = "GRIP:" + str(int(self.grip)) + '\n'
        open_file.writelines((identity, angles, height, grip, '#\n'))

def parse_file(file_lines):
    waypoints = []
    info = {
        "ID": -1,
        "ANGLES": [-1, -1, -1],
        "HEIGHT": -1,
        "GRIP": False
    }
    
    for line in file_lines:
        if line.startswith("ID"):
            try:
                info["ID"] = int(line.split(':')[-1])
            except ValueError as e:
                print(f"Error parsing ID: {line}, error: {e}")
            
        elif line.startswith("ANGLES"):
            try:
                cur_angles = line.split(':')[-1].split(';')
                info["ANGLES"] = (int(cur_angles[0]), int(cur_angles[1]), int(cur_angles[2]))
            except ValueError as e:
                print(f"Error parsing ANGLES: {line}, error: {e}")
                
        elif line.startswith("HEIGHT"):
            try:
                info["HEIGHT"] = int(line.split(':')[-1])
            except ValueError as e:
                print(f"Error parsing HEIGHT: {line}, error: {e}")

        elif line.startswith("GRIP"):
            try:
                info["GRIP"] = bool(int(line.split(':')[-1]))
            except ValueError as e:
                print(f"Error parsing GRIP: {line}, error: {e}")
            
        elif line.startswith("#"):
            if info["ID"] != -1 and all(isinstance(angle, int) for angle in info["ANGLES"]) and info["HEIGHT"] != -1:
                waypoints.append(Waypoint(file_data=info.copy()))
            else:
                print(f"Invalid waypoint data: {info}")

    return waypoints