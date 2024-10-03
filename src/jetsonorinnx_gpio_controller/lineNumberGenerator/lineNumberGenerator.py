import os
import subprocess as sp


ASSETPATH = f"{os.path.dirname(__file__)}/assets/pinToSoC.csv"
OUTDIR = f"{os.path.dirname(__file__)}/output/"

def getJSON():
    with open(ASSETPATH, "r") as f:
        raw = f.read().replace(" ", "")
    
    ret = []
    
    for line in raw.split("\n"):
        if len(line.split(",")) >= 2:
            pin = line.split(",")[0]
            val = line.split(",")[1]
            if(val != "N/A" and pin.isdigit()):
                row = {"pin": int(pin), \
                       "SoC": val \
                        }
                ret.append(row)
    
    return ret

def getLineNumber(soc):
    result = sp.run(["gpiofind", soc], encoding = "utf-8", stdout = sp.PIPE).stdout
    if result == "":
        return -1

    return int(result.split(" ")[1].replace("\n", ""))
        

def getMaxNumber(jsonData):
    maximum = 0
    for row in jsonData:
        if(row["pin"] > maximum):
            maximum = row["pin"]
    return maximum


def generateArrayC(jsonData):
    arr = [-1] * (getMaxNumber(jsonData) + 1)

    for line in jsonData:
        arr[line["pin"]] = getLineNumber(line["SoC"])
    
    out = "const int pin_to_line[] = {"
    for i in range(len(arr)):
        if i != 0:
            out += ","
        out += str(arr[i])
    out += "};"
    
    with open(OUTDIR + "array_C.txt" , "w") as f:
        print(out, file = f)


def generateFunctionC(jsonData):
    arr = [-1] * (getMaxNumber(jsonData) + 1)

    for line in jsonData:
        arr[line["pin"]] = getLineNumber(line["SoC"])
    
    out = "int pin_to_line(int pinNum){\n    const int tmp[] = {"
    for i in range(len(arr)):
        if i != 0:
            out += ","
        out += str(arr[i])
    out += "};"
    out += "\n    return tmp[pinNum];\n}"
    
    with open(OUTDIR + "function_C.txt" , "w") as f:
        print(out, file = f)



def generateArrayPython(jsonData):
    arr = [-1] * (getMaxNumber(jsonData) + 1)

    for line in jsonData:
        arr[line["pin"]] = getLineNumber(line["SoC"])
    
    out = "pinToLine = ["
    for i in range(len(arr)):
        if i != 0:
            out += ","
        out += str(arr[i])
    out += "]"
    
    with open(OUTDIR + "array_Python.txt" , "w") as f:
        print(out, file = f)


def generateFunctionPython(jsonData):
    arr = [-1] * (getMaxNumber(jsonData) + 1)

    for line in jsonData:
        arr[line["pin"]] = getLineNumber(line["SoC"])
    
    out = "def pinToLine(pinNum):\n    tmp = ["
    for i in range(len(arr)):
        if i != 0:
            out += ","
        out += str(arr[i])
    out += "]"
    out += "\n    return tmp[pinNum]\n"
    
    with open(OUTDIR + "function_Python.txt" , "w") as f:
        print(out, file = f)


def main():
    jsonData = getJSON()
    generateArrayC(jsonData)
    generateFunctionC(jsonData)
    generateArrayPython(jsonData)
    generateFunctionPython(jsonData)
    

if __name__ == "__main__":
    main()

        