# Generate C code messages that are compatible with cpp functor based system
import parse
import os,errno
import shutil

with open("../../../../../LICENSE", 'r') as f:
    license = "/*"
    license += f.read()
    license += "*/\n\n"
message_template = license
header_template = license
swig_template = license
message_i_template = license

# clear out an old folder and create a fresh folder of wrapped C message interfaces
destination_dir = '../cMsgCInterface/'
if os.path.exists(destination_dir):
    shutil.rmtree(destination_dir, ignore_errors=True)
try:
    os.makedirs(os.path.dirname(destination_dir))
except OSError as exc:  # Guard against race condition
    if exc.errno != errno.EEXIST:
        raise

# create swig file for C-msg C interface methods
with open(destination_dir + 'cMsgCInterfacePy.i', 'w') as w:
    w.write(swig_template)
swig_template = open(destination_dir + 'cMsgCInterfacePy.i', 'a')

# create the cmake file for the auto-generated C-msg interface files
with open('./CMakeLists.in', 'r') as r:
    cmakeText = r.read()
with open(destination_dir + 'CMakeLists.txt', 'w') as w:
    w.write(cmakeText)

# append all C msg definitions to the message.i file
with open('./message.i.in', 'r') as r:
    messageContent = r.read()
    message_i_template += messageContent
with open(destination_dir + '../message.i', 'w') as w:
    w.write(message_i_template)
for file in os.listdir("../cMsgDefinition"):
    if file.endswith(".h"):
        message_i_template += "\nINSTANTIATE_TEMPLATES(" + os.path.splitext(file)[0] + ")"
message_i_template += '\n\n%include "message.h"\n'
with open(destination_dir + '../message.i', 'w') as w:
    w.write(message_i_template)

with open('./README.in', 'r') as r:
    README = r.read()
message_template += README
header_template += README
swig_template.write(README)
swig_template.write("%module cMsgCInterfacePy\n")

with open('./message.in', 'r') as f:
    message_template += f.read()

with open('./header.in', 'r') as f:
    header_template += f.read()

with open('./swig.in', 'r') as f:
    swig_template_block = f.read()

with open('../message.i', 'r') as fb:
    lines = fb.readlines()

def to_message(struct_data):
    if struct_data:
        struct_data = struct_data.replace(' ', '').split(',')
        struct_name = struct_data[0]
        source_header_file = 'architecture/messaging2/cMsgDefinition/' + struct_name + '.h'
        definitions = message_template.format(type=struct_name)
        header = header_template.format(type=struct_name, structHeader=source_header_file)
        swig_template.write(swig_template_block.format(type=struct_name))
        file_name = destination_dir + struct_name + '_C'
        definitions_file = file_name + '.cpp'
        header_file = file_name + '.h'
        with open(definitions_file, 'w') as w:
            w.write(definitions)
        with open(header_file, 'w') as w:
            w.write(header)
    return

template_call = 'INSTANTIATE_TEMPLATES({:dat})'
for line in lines:
    it = parse.parse(template_call, line, dict(dat=to_message))

