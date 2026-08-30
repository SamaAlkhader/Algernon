import sys

_log = open("mms_debug_log.txt", "a", buffering=1)

def _send_command_and_wait(args):
    """
    Function for commands documented to send back a response line

    """
    line = " ".join(str(a) for a in args)
    _log.write(f"SENT: {line}\n")
    _log.flush()
    sys.stdout.write(line + "\n")
    sys.stdout.flush()
    response = sys.stdin.readline().strip()
    _log.write(f"RECEIVED: {response}\n")
    _log.flush()
    return response

def _send_command_without_response(args):
    """
    Function for commands the official mms explicitly documents as
    getting NO RESPONSE.
    """
    line = " ".join(str(a) for a in args)
    _log.write(f"SENT (no response expected per mms documentation): {line}\n")
    _log.flush()
    sys.stdout.write(line + "\n")
    sys.stdout.flush()

def mazeWidth():
    return int(_send_command_and_wait(["mazeWidth"]))

def mazeHeight():
    return int(_send_command_and_wait(["mazeHeight"]))

def wallFront():
    return _send_command_and_wait(["wallFront"]) == "true"

def wallRight():
    return _send_command_and_wait(["wallRight"]) == "true"

def wallLeft():
    return _send_command_and_wait(["wallLeft"]) == "true"

def wallBack():
    return _send_command_and_wait(["wallBack"]) == "true"

def moveForward(distance=1):
    """
    Per official mms docs:
    Returns 'ack' on success
            'crash' on failure
    """
    args = ["moveForward"]
    if distance != 1:
        args.append(str(distance))
    return _send_command_and_wait(args)

def turnRight():
    return _send_command_and_wait(["turnRight"])

def turnLeft():
    return _send_command_and_wait(["turnLeft"])

def setWall(x,y,direction):
    return _send_command_without_response(["setWall",str(x),str(y),direction])

def clearWall(x,y,direction):
    return _send_command_without_response(["clearWall",str(x),str(y),direction])

def setColor(x,y,color):
    return _send_command_without_response(["setColor",str(x),str(y),color])

def clearColor(x,y):
    return _send_command_without_response(["clearColor",str(x),str(y)])

def clearAllColor():
    return _send_command_without_response(["clearAllColor"])

def setText(x,y,text):
    return _send_command_without_response(["setText",str(x),str(y),text])

def clearText(x,y):
    return _send_command_without_response(["clearText",str(x),str(y)])

def clearAllText():
    return _send_command_without_response(["clearAllText"])

def wasReset():
    return _send_command_and_wait(["wasReset"]) == "true"

def ackReset():
    return _send_command_and_wait(["ackReset"])
