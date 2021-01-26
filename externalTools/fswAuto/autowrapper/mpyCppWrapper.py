import sys
import numpy as np
from Basilisk.architecture import (sim_model, alg_contain)
import copy
import warnings


class CppWrapperClass(object):
    """
        This is the main class handling instrospection of the C modules.
        It generates the C++ wrapper classes (with callbacks to the C algorithms and
        setters and getters for the C module variables) and it also generates the MicroPython integration patch.
    """
    def __init__(self, TheSim, taskActivityDir, simTag, outputPath):
        self.output_store_path = outputPath
        self.wrapper_template = CppWrapClassTemplate()
        self.module_mpy_file = open(self.output_store_path + "/module.hpp", 'w+')

        self.name_replace = TheSim.NameReplace

        self.SelfInit_dict = dict() # {modelTag: SelfInit alg address, moduleID}
        self.Update_dict = dict()  # dictionary D = {modelTag: Update alg address, moduleID}
        self.Reset_dict = dict()  # dictionary D = {modelTag: Reset alg address, moduleID}
        self.fill_dictionaries(simTag, TheSim)

        self.taskIdxDir = self.areTasksInSimTaskList(taskActivityDir, TheSim)
        self.create_class_functions_wrapper(TheSim, taskActivityDir)


    def fill_dictionaries(self, simTag, TheSim):
        TheSimList = dir(eval(simTag))
        i = 0
        for elemName in TheSimList:
            elem = eval(simTag + '.' + elemName)
            if type(elem) == alg_contain.AlgContain:
                self.SelfInit_dict[elem.ModelTag] = (int(elem.getSelfInitAddress()), i)
                self.Update_dict[elem.ModelTag] = (int(elem.getUpdateAddress()), i)
                hasResetAddress = int(elem.getResetAddress())
                if (hasResetAddress):
                    self.Reset_dict[elem.ModelTag] = (hasResetAddress, i)
                i += 1

    def getTaskIndex(self, theSim, taskUse):
        """
            This function returns the key name of the NameReplace dictionary according to the index of the task
            and the index of the model inside the task
        """
        j = 0
        for taskPy in theSim.TaskList:
            if taskUse.TaskName == taskPy.Name:
                return j
            j += 1
        return -1

    def areTasksInSimTaskList(self, taskActivityDir, TheSim):
        print('TASKS BEING PARSED: ')
        taskIdxDir = []
        taskOrderedList = []
        for procIdx in range(0, len(TheSim.TotalSim.processList)):
            locProcList = []
            for i_task in range(len(TheSim.TotalSim.processList[procIdx].processTasks)):
                theTask = TheSim.TotalSim.processList[procIdx].processTasks[i_task]
                taskFound = False
                for ordIdx in range(len(locProcList)):
                    locTask = locProcList[ordIdx]
                    # locTask[0] = taskName
                    # locTask[1] = taskPriority
                    if theTask.taskPriority > locTask[1]:
                        locProcList.insert(ordIdx,
                            [theTask.TaskPtr.TaskName, theTask.taskPriority])
                        taskFound = True
                        break
                if taskFound != True:
                    locProcList.append([theTask.TaskPtr.TaskName, theTask.taskPriority, theTask])
            taskOrderedList.extend(locProcList)

        for i_task in range(0, len(taskOrderedList)):
            # taskOrderedList[i_task][0] = taskName
            # taskOrderedList[i_task][1] = taskPriority
            # taskOrderedList[i_task][2] = theTask
            taskName = taskOrderedList[i_task][0]
            if taskName in taskActivityDir.keys():
                idxUse = self.getTaskIndex(TheSim, taskOrderedList[i_task][2].TaskPtr)
                taskIdxDir.append(idxUse)
                print( i_task, taskName)
        return taskIdxDir

    def getTaskModelKey(self, i_task, i_model):
        key = 'self.TaskList[' + str(i_task) + '].TaskModels[' + str(i_model) + ']'
        return key

    def parseSwigVars(self, list):
        """
            First Parsing Method: Gets rid of all the variables that come after a built-in.
            The methods SelfInit, Update and Restart will always come first, and so will
            the variables that are capitalized (this is based on how "dir()" command works in the Python interpreter).
            Returns a reduced array.
        """
        parsed_array = np.array([])
        length = len(list)
        i = 0
        while i < length:
            if list[i][0] != '_':
                parsed_array = np.append(parsed_array, list[i])
                i += 1
            else:
                break
        return parsed_array

    def evalParsedList(self, list, module):
        """
            Second Parsing Method: Collects all the SwigPyObjects present in the list.
            Only the methods ``SelfInit_(...)``, ``Update_(...)`` and ``Restart_(...)``
            are wrapped by Swig in the ``.i`` files. Therefore they are the only ``SwigPyObjects``.

        :return: a dictionary D = {method, address}
        """
        addressDict = {}
        for methodName in list:
            methodObject = eval('sys.modules["' + module + '"].' + methodName)
            if type(methodObject).__name__ == "SwigPyObject":
                methodAddress = sim_model.getObjectAddress(methodObject)
                addressDict[methodName] = int(methodAddress)
        return addressDict

    def checkMethodType(self, methodName):
        """
            This function checks the method type of the input and returns the corresponding strings.

        :return: methodName: name of the model data algorithm
        """
        str_selfInit = 'SelfInit'
        str_update = 'Update'
        str_reset = 'Reset'

        str_blank = ''  # the methods SelfInit doesn't need the callTime parameter
        str_callTime = ', callTime'  # the methods Reset and Update need an extra parameter for callTine

        if methodName[0:len(str_selfInit)] == str_selfInit:
            return str_selfInit, str_blank
        elif methodName[0:len(str_update)] == str_update:
            return str_update, str_callTime
        elif methodName[0:len(str_reset)] == str_reset:
            return str_reset, str_callTime
        else:
            raise ValueError('Cannot recognize the method. Parse better.')

    def findAddressMatch(self, addressVal, modelTagKey, dict):
        """
            This function makes sure that each algorithm in a data model is matched with the proper algorithm in
            the corresponding model wrap.
            If there's a match, returns the ID of the model.
            Otherwise, an ugly error is raised and the whole program quits.

        :param addressVal: address of the model data algorithm.
        :param modelTagKey: modelTag = key with which to search in ``dict``.
        :param dict: wrap algorithms' address dictionary in which to look for a match. For example, use dict[modelTagKey][0] = model wrap algorithm address or dict[modelTagKey][1] = model ID

        """
        try:
            address = dict[modelTagKey][0]
            if address == addressVal:
                IDVal = dict[modelTagKey][1]
                return IDVal
        except:
            raise ValueError(str(modelTagKey) + ' is not wrapping all the existing algorithms in '
                                                'the corresponding data model. Fix it.')

    def create_class_functions_wrapper(self, TheSim, taskActivityDir):
        for i_task in self.taskIdxDir:
            task = TheSim.TaskList[i_task]
            i_model = 0
            for model in task.TaskModels:
                curr_model_algs_dict = dict()
                key = self.getTaskModelKey(i_task, i_model)
                modelTag = TheSim.NameReplace[key]
                module = model.__module__
                c_struct_name = str(type(model).__name__)
                split_names = module.split('.')
                h_file_name = split_names[-1]
                h_folder_name = split_names[-2]
                header_line = '#include "%s/%s.h"' % (h_folder_name, h_file_name)
                hpp_line = '#include "%s/%s.hpp"' % (h_folder_name, h_file_name)

                sysMod = sys.modules[module]
                dirList = dir(sysMod)
                parsed_dirList = self.parseSwigVars(dirList)
                addressDict = self.evalParsedList(parsed_dirList, module)
                for k, v in addressDict.items():
                    methodType, methodCallTime = self.checkMethodType(k)
                    c_call = k + "(&(this->config_data)" + methodCallTime + ", this->moduleID);"
                    split_arg = methodCallTime.split()
                    arg = ''
                    if split_arg:
                        arg = 'uint64_t %s' % split_arg[-1]
                    curr_model_algs_dict[methodType] = c_call, arg

                self.wrapper_template.create_new_template(modelTag, header_line, c_struct_name, curr_model_algs_dict,
                                                          hpp_line)
                self.autocode_model(model=model, prefix=modelTag)
                self.wrapper_template.join_swigged_data()
                output_file_name = self.output_store_path + "/%s.hpp" % h_file_name
                self.wrapper_template.write_new_class(abs_path_file_name=output_file_name)

                self.wrapper_template.generate_mpy_wrapper_code(self.module_mpy_file)
                self.wrapper_template.reset()

                i_model += 1
        self.module_mpy_file.close()

    def printList(self, input, prefix):
        #print( "input = ", input)
        #print( "len_input = ", len(input))
        for i in range(len(input)):
            prefixIn = prefix + '[' + str(i) + ']'
            if type(input[i]).__name__ == 'list':
                #print( "RECURSION")
                self.printList(input[i], prefixIn)
            else:
                if(input[i] != 0):
                    list_write = prefixIn + ' = ' + str(input[i])+';\n'
                    #print( "str_write = ", list_write)
                    #print( "input_type = ", type(input[i]).__name__)

    def classify_list(self, list, prefix):
        len_list = len(list)
        if type(list[0]).__name__ == 'float':
            callback = "self.wrapper_template.swig_v%s_doubles(prefix)" % len_list
            try:
                eval(callback)
            except:
                warnings.warn("Couldn't evaluate the following callback: %s. Not swigging anything for you here." %
                              callback)

    def autocode_model(self, model, prefix):
        #print( "\nmodel = ", model)
        dir_model = dir(model)
        field_names = copy.copy(dir_model)
        for k in range(0, len(dir_model)):
            fieldName = dir_model[k]
            fieldValue = getattr(model, fieldName)
            fieldTypeName = type(fieldValue).__name__
            fieldTypeFull = str(type(fieldValue))

            if (sys.version_info < (3, 0)):
                # this and __ and SwigPyObject and instancemethod
                if fieldName[0:2] == '__' or fieldName[0:4] == 'this' or \
                        fieldTypeName == 'SwigPyObject' or fieldTypeName == 'instancemethod':
                    field_names.remove(fieldName)
                    continue
                # class
                elif fieldTypeFull[1:6] == 'class':
                    class_prefix = "%s.%s" % (prefix, fieldName)
                    #print( "class_prefix = %s. RECURSION NEEDED." % class_prefix)

                # character array
                elif fieldTypeName == 'str':
                    self.wrapper_template.swig_string(field_name=fieldName)
                elif fieldTypeName == 'int':  # and fieldValue is not 0:
                    if fieldValue == 0:
                        continue
                    else:
                        self.wrapper_template.swig_numeric_value(field_name=fieldName, field_type=fieldTypeName)
                elif fieldTypeName == 'float':  # and fieldValue is not 0.0:
                    if fieldValue == 0:
                        #print( "Value iz zero for fieldName = %s, of type = %s" % (fieldName, fieldTypeName))
                        continue
                    else:
                        self.wrapper_template.swig_numeric_value(field_name=fieldName, field_type='double')

                # handle numeric lists
                elif fieldTypeName == 'list':
                    if all(v == 0 for v in fieldValue):
                        # print( "Value iz zero for fieldName = %s, of type = %s" % (fieldName, fieldTypeName))
                        continue
                    else:
                        self.classify_list(list=fieldValue, prefix=fieldName)

                else:
                    pass
                    #print( "field_name: %s\t field_value: %s\t field_type_name: %s\t field_type_full: %s\t" % (fieldName, fieldValue, fieldTypeName, fieldTypeFull))
            else:
                # this and __ and SwigPyObject and instancemethod
                if fieldName[0:2] == '__' or fieldName[0:4] == 'this' or \
                        fieldTypeName == 'SwigPyObject' or fieldTypeName == 'instancemethod':
                    field_names.remove(fieldName)
                    continue
                # class
                elif 'Basilisk' in fieldTypeFull:
                    class_prefix = "%s.%s" % (prefix, fieldName)
                    # print( "class_prefix = %s. RECURSION NEEDED." % class_prefix)

                # character array
                elif fieldTypeName == 'str':
                    self.wrapper_template.swig_string(field_name=fieldName)
                elif fieldTypeName == 'int':  # and fieldValue is not 0:
                    if fieldValue == 0:
                        continue
                    else:
                        self.wrapper_template.swig_numeric_value(field_name=fieldName, field_type=fieldTypeName)
                elif fieldTypeName == 'float':  # and fieldValue is not 0.0:
                    if fieldValue == 0:
                        # print( "Value iz zero for fieldName = %s, of type = %s" % (fieldName, fieldTypeName))
                        continue
                    else:
                        self.wrapper_template.swig_numeric_value(field_name=fieldName, field_type='double')

                # handle numeric lists
                elif fieldTypeName == 'list':
                    if all(v == 0 for v in fieldValue):
                        # print( "Value iz zero for fieldName = %s, of type = %s" % (fieldName, fieldTypeName))
                        continue
                    else:
                        self.classify_list(list=fieldValue, prefix=fieldName)

                # # non-array variable
                else:
                    pass
                    # print( "field_name: %s\t field_value: %s\t field_type_name: %s\t field_type_full: %s\t" % (fieldName, fieldValue, fieldTypeName, fieldTypeFull))

        #print( "field_names = ", field_names)
        #print( "swigged vars = ", self.wrapper_template.get_swigged_keys())


class SwigNamer(object):
    """
        Class to handler creation of setters and getters (i.e. SWIG functionality)
    """
    def __init__(self, field_name):
        self.config_str = "this->config_data.%s" % field_name
        self.input_str = "new_%s" % field_name
        self.set_call = "Set_%s" % field_name
        self.output_str = "local_%s" % field_name
        self.get_call = "Get_%s" % field_name


class MpyWrapCodeTemplate(object):
    """
        Template class defining the glue-code to include in the MicroPython C++ Wrapper source
    """
    def __init__(self, model_tag, algs_dict, class_name, hpp_line):
        self.wrap_name = 'wrap_%s' % model_tag
        self.class_name = class_name
        self.str_mpy_wrap_header = ""
        self.str_mpy_wrap_struct_names = ""
        self.str_mpy_wrap_init = ""
        self.str_mpy_wrap_props = ""
        self.initialize_code(model_tag, algs_dict, hpp_line)

    def initialize_code(self, model_tag, algs_dict, hpp_line):
        # Create MPy Wrapper Class
        self.str_mpy_wrap_header = hpp_line + '\n'
        struct_name = '%s_FunctionNames' % model_tag
        self.str_mpy_wrap_struct_names = "struct %s" % struct_name + \
                                         "\n{\n" + \
                                         "\tfunc_name_def(SelfInit)\n" + \
                                         "\tfunc_name_def(Update)\n"
        if 'Reset' in algs_dict:
            self.str_mpy_wrap_struct_names += "\tfunc_name_def(Reset)\n"
        self.str_mpy_wrap_struct_names += "};\n"

        self.str_mpy_wrap_init = '\tupywrap::ClassWrapper < %s > %s("%s", mod);\n' \
                                 % (self.class_name, self.wrap_name, self.class_name) + \
                                 '\t%s.DefInit <> ();\n' % self.wrap_name + \
                                 '\t%s.Def < %s::SelfInit > (& %s::SelfInit);\n' % (self.wrap_name, struct_name, self.class_name) + \
                                 '\t%s.Def < %s::Update > (& %s::UpdateState);\n' % (self.wrap_name, struct_name, self.class_name)

        if 'Reset' in algs_dict:
            self.str_mpy_wrap_init += '\t%s.Def < %s::Reset > (& %s::Reset);\n' % (self.wrap_name, struct_name, self.class_name)
        # print( "str_mpy_wrap_struct_names =\n", self.str_mpy_wrap_struct_names)
        # print( "str_mpy_wrap_init =\n", self.str_mpy_wrap_init)


    def add_properties_to_wrap(self, current_vars_swig_dict):
        str_props = ''
        # str_props = '%s.Property("%s", &%s::%s, &%s::%s);\n' % (self.wrap_name, "ModelTag", self.class_name, values[0], self.class_name, values[1])
        for key, values in current_vars_swig_dict.items():
            prop_line = '\t%s.Property("%s", &%s::%s, &%s::%s);\n' % (self.wrap_name, key, self.class_name, values[0],
                                                                  self.class_name, values[1])
            str_props += prop_line
        self.str_mpy_wrap_props = str_props
        return

    def write_module_wrap_code(self, module_mpy_file):
        the_string = self.str_mpy_wrap_header + \
                     self.str_mpy_wrap_struct_names + \
                     self.str_mpy_wrap_init + self.str_mpy_wrap_props + \
                     '\n\n'
        module_mpy_file.write(the_string)


class CppWrapClassTemplate(object):
    """
        Template class defining the C++ wrapper class
    """
    def __init__(self):
        self.str_class_algs = ""
        self.str_class_swig = ""
        self.str_class_config = ""
        self.current_model = ""
        self.current_vars_swig_dict = dict()
        self.current_props_swig_dict = dict()
        self.current_mpy_code_model = None

    def reset(self):
        # print( "CppWrapClassTemplate now resetting(...)")
        self.str_class_algs = ""
        self.str_class_swig = ""
        self.str_class_config = ""
        self.current_model = ""
        self.current_vars_swig_dict = dict()
        self.current_props_swig_dict = dict()
        self.current_mpy_code_model = None

    def get_swigged_keys(self):
        return self.current_vars_swig_dict.keys()

    def generate_mpy_wrapper_code(self, abs_path_file_name):
        self.current_mpy_code_model.add_properties_to_wrap(self.current_props_swig_dict)
        self.current_mpy_code_model.write_module_wrap_code(abs_path_file_name)
        return

    def create_new_template(self, model_tag, header_line, c_struct_name, algs_dict, hpp_line):
        self.current_model = model_tag
        # Create C++ class
        compile_def_name = 'WRAP_%s_HPP' % model_tag
        #print( "compile_def_name = ", compile_def_name)

        str_compile_def = '#ifndef ' + compile_def_name + '\n' + \
                          '#define ' + compile_def_name + '\n\n'

        str_includes = '#include <iostream>\n' + \
                       '#include "utilities/linearAlgebra.h"\n' + \
                       '#include "_GeneralModuleFiles/sys_model.h"\n' + \
                       header_line + '\n'

        class_name = model_tag + "Class"
        # class_name = c_struct_name + "Class"
        str_class = 'class %s: public SysModel {\n' % class_name + \
                         'public: \n' + \
                         '\t%s(){ memset(&this->config_data, 0x0, sizeof(%s));}\n' % (class_name, c_struct_name) + \
                         '\t~%s(){return;}\n' % class_name

        str_callbacks = '\tvoid SelfInit(%s){ %s }\n' % (algs_dict['SelfInit'][1], algs_dict['SelfInit'][0]) + \
                        '\tvoid UpdateState(%s){ %s }\n' % (algs_dict['Update'][1], algs_dict['Update'][0])
        if 'Reset' in algs_dict:
            str_callbacks += '\tvoid Reset(%s){ %s }\n\n' % (algs_dict['Reset'][1], algs_dict['Reset'][0])

        self.str_class_algs = str_compile_def + str_includes + str_class + str_callbacks

        str_c_data = 'private: \n' + \
                     '\t%s config_data;' % c_struct_name

        str_end = '\n}; \n\n#endif'

        self.str_class_config = str_c_data + str_end

        self.swig_model_tag()

        self.current_mpy_code_model = MpyWrapCodeTemplate(model_tag, algs_dict, class_name, hpp_line)


    def join_swigged_data(self):
        str_swig = ""
        for key, values in self.current_vars_swig_dict.items():
            str_swig += values[0] + values[1]
        self.str_class_swig = str_swig
        #print( "str_class_swig = ", self.str_class_swig))

    def write_new_class(self, abs_path_file_name):
        print( "NEW WRAP: output_file_name = ", abs_path_file_name)
        the_string = self.str_class_algs + self.str_class_swig + self.str_class_config
        #print( "\nthe_string=\n", the_string)
        #return
        # Write cpp class
        file_mode = 'w+'  # create file to be written if it doesn't exist
        #alg_class = open(self.output_store_path + '/' + h_file_name + '.hpp', file_mode)
        output_file = open(abs_path_file_name, file_mode)
        output_file.write(the_string)
        output_file.close()

    def swig_model_tag(self):
        field_name = "ModelTag"
        swig = SwigNamer(field_name=field_name)
        # Setter
        setter_str = "\tvoid %s(std::string %s){\n" % (swig.set_call, swig.input_str) + \
                     "\t\tthis->%s = %s;\n" % (field_name, swig.input_str) + \
                     "\t}\n"
        # Getter
        getter_str = "\tstd::string %s() const{\n" % swig.get_call + \
                     "\t\treturn(this->%s);\n" % field_name + \
                     "\t}\n"
        self.current_vars_swig_dict[field_name] = setter_str, getter_str
        self.current_props_swig_dict[field_name] = swig.set_call, swig.get_call

    def swig_string(self, field_name):
        swig = SwigNamer(field_name)
        # Setter
        setter_str = "\tvoid %s(std::string %s){\n" % (swig.set_call, swig.input_str) + \
                     "\t\tmemset(%s, '\\0', sizeof(char) * MAX_STAT_MSG_LENGTH);\n" % swig.config_str + \
                     "\t\tstrncpy(%s, %s.c_str(), %s.length());\n" % (swig.config_str, swig.input_str, swig.input_str) + \
                     "\t}\n"
        # Getter
        getter_str = "\tstd::string %s() const{\n" % swig.get_call + \
                     "\t\tstd::string %s(%s);\n" % (swig.output_str, swig.config_str) + \
                     "\t\treturn(%s);\n" % swig.output_str + \
                     "\t}\n"
        self.current_vars_swig_dict[field_name] = setter_str, getter_str
        self.current_props_swig_dict[field_name] = swig.set_call, swig.get_call
        # print( "\nswig_string(). field_name = ", field_name)
        # print( "setter_str = \n", setter_str)
        # print( "getter_str = \n", getter_str)
        return

    def swig_numeric_value(self, field_name, field_type):
        swig = SwigNamer(field_name)
        # Setter
        setter_str = "\tvoid %s(%s %s) {\n" % (swig.set_call, field_type, swig.input_str) + \
                     "\t\t%s = %s;\n" % (swig.config_str, swig.input_str) + \
                     "\t}\n"
        # Getter
        getter_str = "\t%s %s() const {\n" % (field_type, swig.get_call) + \
                     "\t\treturn (%s);\n" % swig.config_str + \
                     "\t}\n"
        self.current_vars_swig_dict[field_name] = setter_str, getter_str
        self.current_props_swig_dict[field_name] = swig.set_call, swig.get_call

    def swig_v3_doubles(self, field_name):
        swig = SwigNamer(field_name)
        # Setter
        setter_str = "\tvoid %s(std::vector<double>%s) {\n" % (swig.set_call, swig.input_str) + \
                     "\t\tv3Copy(%s.data(), %s);\n" % (swig.input_str, swig.config_str) + \
                     "\t}\n"
        # Getter
        getter_str = "\tstd::vector<double> %s() const {\n" % swig.get_call + \
                     "\t\tstd::vector<double> %s(%s, %s + sizeof(%s) / sizeof(%s[0]) );\n" % \
                     (swig.output_str, swig.config_str, swig.config_str, swig.config_str, swig.config_str) + \
                     "\t\treturn (%s);\n" % swig.output_str + \
                     "\t}\n"

        self.current_vars_swig_dict[field_name] = setter_str, getter_str
        self.current_props_swig_dict[field_name] = swig.set_call, swig.get_call
        # print( "\nswig_v3_doubles(). field_name = ", field_name)
        # print( "setter_str = \n", setter_str)
        # print( "getter_str = \n", getter_str)
        return

    def swig_v9_doubles(self, field_name):
        swig = SwigNamer(field_name)
        # Setter
        setter_str = "\tvoid %s(std::vector<double>%s) {\n" % (swig.set_call, swig.input_str) + \
                     "\t\tm33Copy(RECAST3X3 %s.data(),  RECAST3X3 %s);\n" % (swig.input_str, swig.config_str) + \
                     "\t}\n"
        # Getter
        getter_str = "\tstd::vector<double> %s() const {\n" % swig.get_call + \
                     "\t\tstd::vector<double> %s(%s, %s + sizeof(%s) / sizeof(%s[0]) );\n" % \
                     (swig.output_str, swig.config_str, swig.config_str, swig.config_str, swig.config_str) + \
                     "\t\treturn (%s);\n" % swig.output_str + \
                     "\t}\n"
        self.current_vars_swig_dict[field_name] = setter_str, getter_str
        self.current_props_swig_dict[field_name] = swig.set_call, swig.get_call
        # print( "\nswig_v9_doubles(). field_name = ", field_name)
        # print( "setter_str = \n", setter_str)
        # print( "getter_str = \n", getter_str)
        return
