import yaml

def parse_pose_yaml(input_str):
    try:
        data = yaml.safe_load(input_str)
        x = round(data['pose']['position']['x'], 2)
        y = round(data['pose']['position']['y'], 2)
        z = round(data['pose']['position']['z'], 2)
        oz = round(data['pose']['orientation']['z'], 4)
        ow = round(data['pose']['orientation']['w'], 4)
        return {"x": x, "y": y, "z": z, "oz": oz, "ow": ow}
    except Exception as e:
        print("⚠️  Failed to parse input. Error:", e)
        return None

def main():
    try:
        count = int(input("How many parking spots do you have? "))
    except ValueError:
        print("❌ Please enter a valid number.")
        return

    parking_spot_goals = {}

    for i in range(1, count + 1):
        print(f"\n📥 Enter topic output for parking spot #{i} (press Enter twice to finish):")
        lines = []
        while True:
            line = input()
            if line.strip() == "":
                break
            lines.append(line)
        full_input = "\n".join(lines)
        parsed = parse_pose_yaml(full_input)
        if parsed:
            parking_spot_goals[i] = parsed
        else:
            print(f"❌ Skipping spot #{i} due to parsing error.")

    print("\n✅ Final dictionary:\n")
    print("parking_spot_goals = {")
    for key, val in parking_spot_goals.items():
        print(f"    {key}: {val},")
    print("}")

if __name__ == "__main__":
    main()

