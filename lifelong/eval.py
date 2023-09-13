import numpy

import os
import json
from subprocess import STDOUT, check_output


OUTPUT = "./tmp/tmp.json"
ECBS_EXE = "./ecbs"
PIBT_EXE = "./pibt"


def read_tmp_json_data():
    with open(OUTPUT, "r") as f:
        data_dict = json.load(f)
    tasks_finished = data_dict["num_tasks_finished"]
    runtime = data_dict["runtime"]

    return runtime, tasks_finished


def run_exe(input, exeFile):
    cmd = [exeFile, "-i", input, "-o", OUTPUT]
    flag=True
    try:
        output = check_output(cmd, stderr=STDOUT, timeout=95).decode('utf-8')
    except:
        print("out of time")
        flag=False
    return flag


def dumpToCsv(header, data, csvFile):
    with open(csvFile, "w") as f:
        numItems = len(header)
        numAgents = len(data[0])
        for k in range(numItems):
            f.write(header[k])
            if k != numItems-1:
                f.write(",")
            else:
                f.write("\n")
        # print(numItems,numAgents,len(data),len(data[0]))
        for i in range(numAgents):
            for k in range(numItems):
                f.write(str(data[k][i]))
                if k != numItems-1:
                    f.write(",")
                else:
                    f.write("\n")


def evalCBS():
    input_folder = "./instances/lifelong/random/"
    outputCSV = "./data/ecbs_obs.csv"
    header = ["num_agents", "runtime", "tasks_finished"]

    runtimeData = []
    tasks_data = []

    num_agents = [5, 10, 15, 20, 25]
    agentsData = []
    num_instances = 10
    for n in num_agents:
        runtime_sum = 0
        tasks_sum = 0
        succ_sum = 0
        for k in range(num_instances):
            instance = input_folder + "agents_"+str(n)+"_"+str(k)+".json"
            print(instance)
            run_exe(instance, ECBS_EXE)
            try:
                runtime, tasks_finished = read_tmp_json_data()

                runtime_sum += runtime
                tasks_sum += tasks_finished
                succ_sum += 1

            except:
                # failed
                pass

            try:
                # break
                os.remove(OUTPUT)
            except:
                pass
        if succ_sum == 0:
            succ_sum += 1e-6

        runtimeData.append(runtime_sum/succ_sum)
        tasks_data.append(tasks_sum/succ_sum)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, runtimeData, tasks_data], outputCSV)



def evalPIBT():
    input_folder = "./instances/lifelong/random/"
    outputCSV = "./data/pibt.csv"
    header = ["num_agents", "runtime", "tasks_finished"]

    runtimeData = []
    tasks_data = []

    num_agents = [5,10,15,20, 25]
    agentsData = []
    num_instances = 10
    for n in num_agents:
        runtime_sum = 0
        tasks_sum = 0
        succ_sum = 0
        for k in range(num_instances):
            instance = input_folder + "agents_"+str(n)+"_"+str(k)+".json"
            print(instance)
            count=0
            flag=False
            while flag==False and count<20:
                flag=run_exe(instance, PIBT_EXE)
                count=count+1
                if flag==False:
                    print("re-run")
            try:
                runtime, tasks_finished = read_tmp_json_data()

                runtime_sum += runtime
                tasks_sum += tasks_finished
                succ_sum += 1

            except:
                # failed
                pass

            try:
                # break
                os.remove(OUTPUT)
            except:
                pass
        if succ_sum == 0:
            succ_sum += 1e-6

        runtimeData.append(runtime_sum/succ_sum*5000/2000)
        tasks_data.append(tasks_sum/succ_sum*5000/2000)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, runtimeData, tasks_data], outputCSV)



if __name__ == "__main__":
    # evalCBS()
    # evalECBS()
    evalPIBT()
    # evalCBS()
