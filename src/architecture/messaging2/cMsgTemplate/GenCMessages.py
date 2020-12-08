# Generate C code messages that are compatible with cpp functor based system
import parse
import os,errno
import shutil
from sys import platform

with open("../../../../LICENSE", 'r') as f:
    license = "/*"
    license += f.read()
    license += "*/\n\n"
messaging2_template = license
header_template = license

# clear out an old folder and create a fresh folder of wrapped C message interfaces
autoSourceDestDir = '../../../../dist3/autoSource/'
if os.path.exists(autoSourceDestDir):
    shutil.rmtree(autoSourceDestDir, ignore_errors=True)
try:
    os.makedirs(os.path.dirname(autoSourceDestDir))
except OSError as exc:  # Guard against race condition
    if exc.errno != errno.EEXIST:
        raise

destination_dir = autoSourceDestDir + 'cMsgCInterface/'
os.makedirs(os.path.dirname(destination_dir))


# create swig file for C-msg C interface methods
swig_template = open(autoSourceDestDir + 'cMsgCInterfacePy.auto.i', 'w')


# append all C msg definitions to the dist3/autoSource/messaging2.auto.i file that is imported into messaging2.i
messaging2_header_i_template = ""
if platform == "linux" or platform == "linux2":
    messaging2_header_i_template = "#define SWIGWORDSIZE64\n"
with open(autoSourceDestDir + 'messaging2.header.auto.i', 'w') as w:
    w.write(messaging2_header_i_template)

messaging2_i_template = "//C messages:"
for file in os.listdir("../../msgPayloadDefC"):
    if file.endswith(".h"):
        msgName = (os.path.splitext(file)[0])[:-7]
        messaging2_i_template += "\nINSTANTIATE_TEMPLATES(" + msgName + ", " \
                                 + msgName + "Payload, ../architecture/msgPayloadDefC)"
with open(autoSourceDestDir + 'messaging2.auto.i', 'w') as w:
    w.write(messaging2_i_template)


with open('./README.in', 'r') as r:
    README = r.read()
messaging2_template += README
header_template += README

with open('./msg_C.cpp.in', 'r') as f:
    messaging2_template += f.read()

with open('./msg_C.h.in', 'r') as f:
    header_template += f.read()

with open('./cMsgCInterfacePy.i.in', 'r') as f:
    swig_template_block = f.read()

with open(autoSourceDestDir + 'messaging2.auto.i', 'r') as fb:
    lines = fb.readlines()

# The following cpp message definitions must be included after the `lines` variable is set above.
# We only need to create Python interfaces to C++ messages, not C wrappers.
messaging2_i_template = "\n\n//C++ messages:"
for file in os.listdir("../../msgPayloadDefCpp"):
    if file.endswith(".h"):
        msgName = (os.path.splitext(file)[0])[:-7]
        messaging2_i_template += "\nINSTANTIATE_TEMPLATES(" + msgName + ", " \
                                 + msgName + "Payload, ../architecture/msgPayloadDefCpp)"
with open(autoSourceDestDir + 'messaging2.auto.i', 'a') as w:
    w.write(messaging2_i_template)


def to_message(struct_data):
    if struct_data:
        struct_data = struct_data.replace(' ', '').split(',')
        struct_name = struct_data[0]
        source_header_file = '../architecture/msgPayloadDefC/' + struct_name + 'Payload.h'
        definitions = messaging2_template.format(type=struct_name)
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
