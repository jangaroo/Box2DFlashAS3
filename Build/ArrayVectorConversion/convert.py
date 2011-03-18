import os
import re
def find_as_files(root):
    for dirpath,dirnames,filenames in os.walk(root):
        for filename in filenames:
            if filename[-3:]==".as":
                yield os.path.join(dirpath,filename)

def write_file(filename, text):
    print("Changing "+filename)
    file = open(filename, "w")
    file.write(text)
    file.close()



def convert_file(filename, convert):
    file = open(filename, 'r')
    text = file.read()
    file.close()

    changed = False
    while True:
        new_text = convert(text)
        if new_text is None:
            break
        else:
            changed = True
            text = new_text
        
    if changed:
        write_file(filename, text)
        
def convert_to_vector(text):
    changed = [False]
    def replace(match):
        s = match.start(0)
        changed[0] = True
        return "Vector.<"+match.group(1)+">"
    text = re.sub(r"Array/\*(.*?)\*/", replace,text)
    if changed[0]:
        return text

def convert_to_array(text):
    new_text = ""
    old_pos = 0
    for match in re.finditer(r"Vector\.<", text):
        if match.start(0) < old_pos:
            continue
        s = match.start(0)
        if text[s-2:s]=="/*":
            continue
        start = end = match.end(0)
        angle_balance = 1
        while angle_balance > 0:
            if   text[end]==">":
                angle_balance -= 1
            elif text[end]=="<":
                angle_balance += 1
            end += 1
            
        new_text += text[old_pos:match.start(0)]
        new_text += "Array/*"+text[start:end-1]+"*/"
        old_pos = end
        
    if old_pos >0:
        new_text += text[old_pos:]
        return new_text
    
def get_sources():
    for filename in find_as_files("../../box2d/src/main/joo"):
        yield filename
    for filename in find_as_files("../../box2d-examples/src/main/joo"):
        yield filename
    for filename in find_as_files("../../box2d-contrib/src/main/joo"):
        yield filename
    for filename in find_as_files("../../box2d-legacy/src/main/joo"):
        yield filename
        
def usage():
    print("""\
convert.py [-v] [-a] [<filename>]
-v        Convert to use Vectors (Flash 10)
-a        Convert to use Arrays (Flash 9)
[<file>]  Convert a specific file (default is all files)
""")
import sys
import getopt

def main(argv): 
    try: 
        opts, args = getopt.getopt(argv, "avh", [])
    except getopt.GetoptError:
        usage()
        sys.exit(2)
    opts = dict(opts)
    if "-h" in opts:
        usage()
        sys.exit(2)
    to_vector = '-a' not in opts
    interactive = False
    if '-a' not in opts and '-v' not in opts:
        r = " "
        if len(args)==0:
            usage()
        print
        interactive = True
        while r not in "av":
            r = raw_input("Convert to vectors or arrays (v/a)?")
        to_vector = r == "v"
    
    
    conversion = None
    if to_vector:
        conversion = convert_to_vector
    else:
        conversion = convert_to_array
    
    if len(args) == 0:
        args = get_sources()
        
    for filename in args:
        convert_file(filename, conversion)
        
if __name__ == "__main__":
    main(sys.argv[1:])