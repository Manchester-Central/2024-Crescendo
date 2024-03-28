import requests
import json

URL = "http://limelight-back.local:5807/results"

s = requests.Session()
with open("limelightdata.csv", 'a') as fout:
    while True:
        with s.get(URL) as resp:
            jsn = json.loads(resp.text)["Results"]
            print(jsn["botpose_wpiblue"])
            resp = resp.text.replace("\n", "")
            fout.write(', '.join([
                                    str(jsn["botpose_avgarea"]),
                                    str(jsn["botpose_avgdist"]),
                                    str(jsn["botpose_span"]),
                                    str(jsn["botpose_tagcount"]),
                                    str(', '.join([str(i) for i in jsn["botpose_wpiblue"]]))
                                  ])
            )
            fout.write('\n')
