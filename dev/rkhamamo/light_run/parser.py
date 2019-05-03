import csv, sys, json

csv_file = sys.argv[1]
end = len(csv_file) - 3
json_file = csv_file[0:end]
json_file += 'json'

def read_CSV(csv_file, json_file):
    csv_rows = []
    with open(csv_file) as csvfile:
        reader = csv.DictReader(csvfile)
        field = reader.fieldnames
        for row in reader:
            csv_rows.extend([{field[i]:row[field[i]] for i in range(len(field))}])
        convert_write_json(csv_rows, json_file)

def convert_write_json(data, json_file):
    with open(json_file, "w") as f:
        f.write(json.dumps(data, sort_keys=False, indent=4, separators=(',', ': ')))


read_CSV(csv_file,json_file)
