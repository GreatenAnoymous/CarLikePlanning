import yaml
import json

def yaml_to_json(yaml_data):
    # Load YAML data
    data = yaml.load(yaml_data, Loader=yaml.FullLoader)

    # Extract necessary information from the YAML data
    dimensions = data['map']['dimensions']
    obstacles = data['map']['obstacles']
    agents = data['agents']

    starts = [agent['start'] for agent in agents]
    goals = [agent['goal'] for agent in agents]

    # Create the JSON output
    json_output = {
        'xmax': dimensions[0],
        'ymax': dimensions[1],
        'obstacles': obstacles,
        'starts': starts,
        'goals': goals
    }

    return json_output


def test():
    # Load YAML file
    with open("input.yaml", "r") as file:
        yaml_data = file.read()

    # Convert YAML to JSON
    json_output = yaml_to_json(yaml_data)

    # Write JSON to a file
    with open("output.json", "w") as file:
        json.dump(json_output, file, indent=2)

if __name__ == "__main__":
    test()
