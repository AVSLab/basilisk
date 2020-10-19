# Generate C code messages that are compatible with cpp functor based system
import parse
import os,errno

with open("../../../../LICENSE", 'r') as f:
    license = "/*"
    license += f.read()
    license += "*/\n\n"
message_template = license
header_template = license
swig_template = license

destination_dir = './cMessages/'
if not os.path.exists(os.path.dirname(destination_dir)):
    try:
        os.makedirs(os.path.dirname(destination_dir))
    except OSError as exc:  # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

with open(destination_dir + 'cMessages.i', 'w') as w:
    w.write(swig_template)
swig_template = open(destination_dir + 'cMessages.i', 'a')

with open('cMessageTemplate/README_template', 'r') as r:
    README = r.read()
message_template += README
header_template += README
swig_template.write(README)
swig_template.write("%module cMessages\n")

with open('./cMessageTemplate/message_template', 'r') as f:
    message_template += f.read()

with open('./cMessageTemplate/header_template', 'r') as f:
    header_template += f.read()

with open('./cMessageTemplate/swig_template', 'r') as f:
    swig_template_block = f.read()

with open('./message.i', 'r') as fb:
    lines = fb.readlines()

def to_message(struct_data):
    if struct_data:
        struct_data = struct_data.replace(' ', '').split(',')
        struct_name = struct_data[0]
        source_dir = "../../../" + struct_data[2] + '/'
        source_header_file = source_dir + struct_data[1] + '.h'
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

