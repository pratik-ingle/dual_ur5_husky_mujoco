#!/usr/bin/python
import sys

def main(args):
    filename = sys.argv[1]
    file = open(filename, 'r')
    reason = sys.argv[2]
    if reason is '0':
        reason = "Adjacent"
    elif reason is '1':
        reason = "Never"

    output = ""
    for line in file:
        outputs = line.split('-')
        col1 = outputs[0].strip()
        col2 = outputs[1].strip()
        if col1 is not None and col2 is not None:
            template= "<disable_collisions link1=\"%s\" link2=\"%s\" reason=\"%s\" />" % (col1, col2, reason)
            output += (template + "\n")
    
    print output
                

if __name__ == "__main__":
    main(sys.argv)    
