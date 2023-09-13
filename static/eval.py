import numpy

import os
import json
from subprocess import STDOUT, check_output


OUTPUT = "./tmp/tmp.json"
ECBS_EXE = "./CL-ECBS"
CBS_EXE = "./CL-CBS"
PIBT_EXE = "./PIBT"
SH_EXE="./SH_Astar"


def read_tmp_json_data():
    with open(OUTPUT, "r") as f:
        data_dict = json.load(f)
    makespan = data_dict["makespan"]
    cost = data_dict["cost"]
    flow_time = data_dict["flowtime"]
    runtime = data_dict["runtime"]
    arrived_agents = 1
    solved = True
    try:
        solved = data_dict["solved"]
        arrived_agents = data_dict["arrivalRate"]
    except:
        pass

    return makespan, cost, runtime, flow_time, runtime, arrived_agents, solved


def run_exe(input, exeFile):
    cmd = [exeFile, "-i", input, "-o", OUTPUT]
    try:
        output = check_output(cmd, stderr=STDOUT, timeout=65).decode('utf-8')
    except:
        print("out of time")


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
    input_folder = "./instances/one_shot/empty/"
    outputCSV = "./data/sh_empty.csv"
    header = ["num_agents", "makespan", "cost", "flowtime",
              "runtime", "success_rate", "arrivalRate"]
    makespanData = []
    costData = []
    flowtimeData = []
    runtimeData = []
    rateData = []
    arrivalData = []
    num_agents = [10, 20, 30, 40, 50, 60]
    agentsData = []
    num_instances = 30
    for n in num_agents:
        makespan_sum = 0
        cost_sum = 0
        flowtime_sum = 0
        runtime_sum = 0
        succ_sum = 0
        arrivalRate = 0
        for k in range(num_instances):
            instance = input_folder + "agents_"+str(n)+"_"+str(k)+".json"
            print(instance)
            run_exe(instance, SH_EXE)
            try:
                makespan, cost, runtime, flow_time, runtime, arrived_agents, solved = read_tmp_json_data()
                makespan_sum += makespan
                cost_sum += cost
                runtime_sum += runtime
                flowtime_sum += flow_time
                if makespan < 300:
                    succ_sum += 1
                arrivalRate += arrived_agents
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
        makespanData.append(makespan_sum/succ_sum)
        costData.append(cost_sum/succ_sum)
        flowtimeData.append(flowtime_sum/succ_sum)
        runtimeData.append(runtime_sum/succ_sum)
        arrivalData.append(arrivalRate/succ_sum)
        rateData.append(succ_sum/num_instances)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, makespanData, costData,
                  flowtimeData, runtimeData, rateData, arrivalData], outputCSV)


def evalECBS():
    input_folder = "./instances/one_shot/random/"
    outputCSV = "./data/sh_random.csv"
    header = ["num_agents", "makespan", "cost", "flowtime",
              "runtime", "success_rate", "arrivalRate"]
    makespanData = []
    costData = []
    flowtimeData = []
    runtimeData = []
    rateData = []
    arrivalData = []
    num_agents = [10, 20, 30, 40, 50, 60]
    agentsData = []
    num_instances = 30
    for n in num_agents:
        makespan_sum = 0
        cost_sum = 0
        flowtime_sum = 0
        runtime_sum = 0
        succ_sum = 0
        arrivalRate = 0
        for k in range(num_instances):
            instance = input_folder + "agents_"+str(n)+"_"+str(k)+".json"
            print(instance)
            run_exe(instance, SH_EXE)
            try:
                makespan, cost, runtime, flow_time, runtime, arrived_agents, solved = read_tmp_json_data()
                makespan_sum += makespan
                cost_sum += cost
                runtime_sum += runtime
                flowtime_sum += flow_time
                if makespan < 300:
                    succ_sum += 1
                arrivalRate += arrived_agents
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
        makespanData.append(makespan_sum/succ_sum)
        costData.append(cost_sum/succ_sum)
        flowtimeData.append(flowtime_sum/succ_sum)
        runtimeData.append(runtime_sum/succ_sum)
        arrivalData.append(arrivalRate/succ_sum)
        rateData.append(succ_sum/num_instances)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, makespanData, costData,
                  flowtimeData, runtimeData, rateData, arrivalData], outputCSV)


def evalPIBT():
    input_folder = "./instances/one_shot/empty/"
    outputCSV = "./data/pibtnocost.csv"
    header = ["num_agents", "makespan", "cost", "flowtime",
              "runtime", "success_rate", "arrivalRate"]
    makespanData = []
    costData = []
    flowtimeData = []
    runtimeData = []
    rateData = []
    arrivalData = []
    num_agents = [10, 20, 30, 40, 50, 60]
    agentsData = []
    num_instances = 30
    for n in num_agents:
        makespan_sum = 0
        cost_sum = 0
        flowtime_sum = 0
        runtime_sum = 0
        succ_sum = 0
        arrivalRate = 0
        for k in range(num_instances):
            instance = input_folder + "agents_"+str(n)+"_"+str(k)+".json"
            print(instance)
            run_exe(instance, PIBT_EXE)
            try:
                makespan, cost, runtime, flow_time, runtime, arrived_agents, solved = read_tmp_json_data()
                if solved:
                    succ_sum += 1

                    makespan_sum += makespan
                    cost_sum += cost
                    runtime_sum += runtime
                    flowtime_sum += flow_time

                    arrivalRate += arrived_agents
                else:
                    arrivalRate += arrived_agents
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
        makespanData.append(makespan_sum/succ_sum)
        costData.append(cost_sum/succ_sum)
        flowtimeData.append(flowtime_sum/succ_sum)
        runtimeData.append(runtime_sum/succ_sum)
        arrivalData.append(arrivalRate/num_instances)
        rateData.append(succ_sum/num_instances)
        agentsData.append(n)
        dumpToCsv(header, [agentsData, makespanData, costData,
                  flowtimeData, runtimeData, rateData, arrivalData], outputCSV)


if __name__ == "__main__":
    # evalCBS()
    # evalECBS()
    evalPIBT()

