import subprocess
import sys
import os
import getopt
from jinja2 import Environment, FileSystemLoader, select_autoescape

# Helper functions

def run_sys_cmd(cmd, cwd=None):

    # Helper function for running Linux commands

    subprocess.run(cmd, cwd=cwd, stderr=sys.stderr, stdout=sys.stdout, shell=True)
    return

def process_msg_file(filename, io_map):

    # Generates the appropriate input and output ROS2 message files given the user logic input and output signals
    
    f = open("output/" + filename + "-int.msg", "w")
    for signal_name in io_map.keys():
        is_arr = io_map[signal_name]["arr"]
        signal_type = io_map[signal_name]["type"]
        num_bits = str(io_map[signal_name]["n_bits"])
        signed = io_map[signal_name]["signed"]
        ros2_type = ""
        if not signed:
            ros2_type = "u"
        ros2_type += signal_type
        ros2_type += num_bits
        if is_arr:
            n_elem = str(io_map[signal_name]["n_elem"])
            ros2_type += "[" + n_elem + "]"
        f.write(ros2_type + " " + signal_name + "\n")
    f.close()
    return

def usage():
    # Prints the optional commands for the script
    print("\n[Forest]: usage: python3 forest.py [-h] [-t] [-g -i ninputs -o noutputs]")
    print("\n-h or --help: Prints the usage statement for the script")
    print("\n-t or --test: Generates simple talker and listener nodes along with the FPGA ROS node")
    print("\n-g or --genconfig: Generates a template config file to be used by the script")
    print("\n-i or --ninputs: Number of input signals for the template config file")
    print("\n-o or --noutputs: Number of output signals for the template config file\n")

def check_in_map_validity(in_map):
    # Check the input map generated after parsing the config file, and
    # verify its validity

    n_axis = 0
    err_found = False
    wrong_in = ""
    for input_signal in in_map.keys():
        properties = in_map[input_signal].keys()
        if "protocol" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "type" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "n_bits" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "signed" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "arr" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "n_elem" not in properties:
            err_found = True
            wrong_in = input_signal
        elif "addr" not in properties:
            err_found = True
            wrong_in = input_signal
        
        if in_map[input_signal]["protocol"] == "stream":
            n_axis+=1

        if n_axis > 1:
            print("\n[Forest]: Error! Only 1 input signal can use the AXI-Stream protocol\n")
            sys.exit(1)

    if err_found:
        print("\n[Forest]: Input map is not valid. Make sure the input definition is correct. Error at signal {}\n". format(wrong_in))
        sys.exit(1)

def check_out_map_validity(out_map):

    # Check the output map generated after parsing the config file, and
    # verify its validity

    n_axis = 0
    err_found = False
    wrong_out = ""
    for output_signal in out_map.keys():
        properties = out_map[output_signal].keys()
        if "protocol" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "type" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "n_bits" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "signed" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "arr" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "n_elem" not in properties:
            err_found = True
            wrong_out = output_signal
        elif "addr" not in properties:
            err_found = True
            wrong_out = output_signal

        if not out_map[output_signal]["arr"]:
            print("\n[Forest]: Error! All output signals must be an array types at signal {}\n".format(output_signal))
            sys.exit(1)
        
        if out_map[output_signal]["protocol"] == "stream":
            n_axis+=1

        if n_axis > 1:
            print("\n[Forest]: Error! Only 1 output signal can use the AXI-Stream protocol\n")
            sys.exit(1)

    if err_found:
        print("\n[Forest]: Output map is not valid. Make sure the output definition is correct. Error at signal {}\n". format(wrong_out))
        sys.exit(1)


# Rendering functions

def render_config_file(env, n_config_inputs, n_config_outputs):

    # Render config.forest template when running in genconfig mode
    
    config_file = env.get_template("config.forest.jinja2")
    f = open("config.forest", "w")
    f.write(config_file.render(n_in=n_config_inputs, n_out=n_config_outputs))
    f.close()

    return

def render_int_package_xml(env, prj):

    # Render package.xml for the interface package
    
    package_xml = env.get_template("package-int.xml.jinja2")
    f = open("output/package-int.xml", "w")
    f.write(package_xml.render(prj_name=prj))
    f.close()

    return

def render_int_cmakelists_txt(env, prj):

    # Render CMakeLists.txt for the interface package

    cmake_txt = env.get_template("CMakeLists-int.txt.jinja2")
    f = open("output/CMakeLists-int.txt", "w")
    f.write(cmake_txt.render(prj_name=prj))
    f.close()

    return

def render_node_package_xml(env, prj):

    # Render package.xml for the node package

    package_xml = env.get_template("package-node.xml.jinja2")
    f = open("output/package-node.xml", "w")
    f.write(package_xml.render(prj_name=prj))
    f.close()

    return

def render_node_setup_py(env, prj, test_enabled):

    # Render setup.py for the node package

    setup_py = env.get_template("setup.py.jinja2")
    f = open("output/setup-node.py", "w")
    f.write(setup_py.render(prj_name=prj, test_enabled=test_enabled))
    f.close()

    return

def render_node_ros_fpga_lib_py(env, bit_file, ip_name, in_map, out_map, has_axis_in, has_axis_out):

    # Render ros_fpga_lib.py for the node package

    ros_fpga_lib_py = env.get_template("ros_fpga_lib.py.jinja2")
    f = open("output/ros_fpga_lib-node.py", "w")
    f.write(ros_fpga_lib_py.render(bit_file=bit_file, ip_name=ip_name, in_map=in_map, out_map=out_map, has_axis_in=has_axis_in, has_axis_out=has_axis_out))
    f.close()

    return

def render_node_fpga_node_py(env, prj, qos, in_signals, out_signals, has_axis_in, has_axis_out):

    # Render fpga_node.py for the node package

    fpga_node_py = env.get_template("fpga_node.py.jinja2")
    f = open("output/fpga_node-node.py", "w")
    f.write(fpga_node_py.render(qos=qos, prj_name=prj, in_signals=in_signals, out_signals=out_signals, has_axis_in=has_axis_in, has_axis_out=has_axis_out))
    f.close()

    return

def render_test_talker(env, prj, qos, in_map):

    # Render the message generation file for the node package

    talker_py = env.get_template("talker.py.jinja2")
    f = open("output/talker-node.py", "w")
    f.write(talker_py.render(prj_name=prj, qos=qos, in_map=in_map))
    f.close()

    return

def render_test_listener(env, prj, qos):

    # Render the message reader file for the node package

    listener_py = env.get_template("listener.py.jinja2")
    f = open("output/listener-node.py", "w")
    f.write(listener_py.render(prj_name=prj, qos=qos))
    f.close()

    return

# Package generation functions

def create_msg_pkg(dev_ws, prj, in_map, out_map):

    # Creates and builds the interface package

    print("\n[Forest]: Generating the ROS2 package for the FPGA node messages\n")

    # Create ROS2 package for the ROS FPGA messages (interface)

    pkg_name = prj + '_interface'

    run_sys_cmd(['ros2 pkg create --build-type ament_cmake ' + pkg_name], cwd=dev_ws+'src/')

    # Create the FPGA message files
    
    process_msg_file("FpgaIn", in_map)
    process_msg_file("FpgaOut", out_map)

    run_sys_cmd(['mkdir -p msg'], cwd= dev_ws+ 'src/' + pkg_name)
    
    run_sys_cmd(['cp output/FpgaIn-int.msg ' + dev_ws + 'src/' + pkg_name + '/msg/FpgaIn.msg'])
    run_sys_cmd(['cp output/FpgaOut-int.msg ' + dev_ws + 'src/' + pkg_name + '/msg/FpgaOut.msg'])

    # Copy modified interface CMakeLists.txt

    run_sys_cmd(['cp output/CMakeLists-int.txt ' + dev_ws + 'src/' + pkg_name + '/CMakeLists.txt'])

    # Copy modified interface package.xml

    run_sys_cmd(['cp output/package-int.xml ' + dev_ws + 'src/' + pkg_name + '/package.xml'])

    # Build message package

    print("\n[Forest]: Building the FPGA ROS2 node messages package\n")

    run_sys_cmd(['colcon build --packages-select ' + pkg_name], cwd=dev_ws)

    return

def create_fpga_node_pkg(dev_ws, prj, test_enabled):

    # Creates and builds the node package

    # Create ROS2 package for the ROS FPGA node

    print("\n[Forest]: Generating the ROS2 package for the FPGA node\n")

    pkg_name = prj + '_fpga_node'

    run_sys_cmd(['ros2 pkg create --build-type ament_python ' + pkg_name], cwd=dev_ws+'src/')

    # Copy modified node package.xml

    run_sys_cmd(['cp output/package-node.xml ' + dev_ws + 'src/' + pkg_name + '/package.xml'])

    # Copy modified node setup.py

    run_sys_cmd(['cp output/setup-node.py ' + dev_ws + 'src/' + pkg_name + '/setup.py'])

    # Copy modified node fpga_node.py

    run_sys_cmd(['cp output/fpga_node-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/fpga_node.py'])

    # Copy modified node ros_fpga_lib.py

    run_sys_cmd(['cp output/ros_fpga_lib-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/ros_fpga_lib.py'])

    # If running in test generation mode, copy the test nodes as well
    
    if test_enabled:
        run_sys_cmd(['cp output/talker-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/talker.py'])
        run_sys_cmd(['cp output/listener-node.py ' + dev_ws + 'src/' + pkg_name + '/' + pkg_name + '/listener.py'])

    # Build the FPGA node package
    
    print("\n[Forest]: Building the FPGA ROS2 node package\n")
    run_sys_cmd(['colcon build --packages-select ' + pkg_name], cwd=dev_ws)

    return

# Main

def main():

    # Setup

    env = Environment(
        loader=FileSystemLoader('./templates/')
    )
 
    test_enabled = False

    gen_config_file = False

    n_config_inputs = 0

    n_config_outputs = 0

    # Parse command line arguments

    arg_list = sys.argv[1:] 
    # Options 
    short_options = "htgi:o:" 
    long_options = ["help", "test", "genconfig", "ninputs=", "noutputs="] 
    try: 
        args, values = getopt.getopt(arg_list, short_options, long_options)    
        # checking each argument 
        for curr_arg, curr_val in args:
            if curr_arg in ("-h", "--help"): 
                usage()
                sys.exit()
            if curr_arg in ("-t", "--test"): 
                print ("\n[Forest]: Test nodes will be generated\n")
                test_enabled = True
                break
            if curr_arg in ("-g", "--genconfig"): 
                print("\n[Forest]: Config file generation mode\n")
                gen_config_file = True
            if curr_arg in ("-i", "--ninputs"):
                if gen_config_file == True:
                    n_config_inputs = int(curr_val)
                else:
                    usage()
                    sys.exit(1)
            if curr_arg in ("-o", "--noutputs"):
                if gen_config_file == True:
                    n_config_outputs = int(curr_val)
                else:
                    usage()
                    sys.exit(1)
                        
    except getopt.error as err: 
        print(str(err)) 
        usage()
        sys.exit(1)

    if gen_config_file == True:
        if n_config_inputs == 0 or n_config_outputs == 0:
            print("\n[Forest]: Error! Need the number of input and output signals\n")
            usage()
        else:
            gen_new = True
            # Check if config.forest file already exists
            if os.path.isfile("config.forest"):
                overwrite = input("\n[Forest]: config.forest file already exists. Overwrite it? (y/n)")
                if "y" in overwrite:
                    gen_new = True
                else:
                    gen_new = False
            if gen_new:
                # Render config file template
                render_config_file(env, n_config_inputs, n_config_outputs)
        sys.exit()

    # Parse config file

    prj = ""
    bit_file = ""
    ip_name = ""
    dev_ws = ""
    in_map = {}
    out_map = {}
    has_axis_in = False
    has_axis_out = False

    reading_input = False
    reading_output = False
    
    try:
        f = open("config.forest", "r")
    except:
        print("\n[Forest]: Config file could not be opened! I need a config.forest file in the same directory as forest.py\n")
        sys.exit(1)
    config_data = f.read().splitlines()
    for line in config_data:
        # Parse input and assign values to variables
        if not (line.startswith("*") or line.startswith("/") or not line):
            key, value = line.split(":")
            key = key.strip()
            value = value.strip()

            # Setup Information
            if (key == "Forest project name"):
                prj = value
            elif (key == "Absolute ROS2 dev_ws path"):
                dev_ws = value
            elif (key == "Absolute FPGA .bit file path"):
                bit_file = value
            elif (key == "User IP name"):
                ip_name = value

            # Inputs
            elif (key == "Input name"):
                in_map[value] = {}
                reading_input = True
                in_name = value
            elif (key == "Protocol" and reading_input):
                in_map[in_name]["protocol"] = value.lower()
                if "stream" in value.lower():
                    has_axis_in = True
            elif (key == "Type" and reading_input):
                if value == "uint8":
                    in_map[in_name]["type"] = "int"
                    in_map[in_name]["n_bits"] = 8
                    in_map[in_name]["signed"] = False
                    in_map[in_name]["arr"] = False
                    in_map[in_name]["n_elem"] = 1
                elif value == "int32":
                    in_map[in_name]["type"] = "int"
                    in_map[in_name]["n_bits"] = 32
                    in_map[in_name]["signed"] = True
                    in_map[in_name]["arr"] = False
                    in_map[in_name]["n_elem"] = 1
                elif value == "float32":
                    in_map[in_name]["type"] = "float"
                    in_map[in_name]["n_bits"] = 32
                    in_map[in_name]["signed"] = True
                    in_map[in_name]["arr"] = False
                    in_map[in_name]["n_elem"] = 1
                elif value == "float64":
                    in_map[in_name]["type"] = "float"
                    in_map[in_name]["n_bits"] = 64
                    in_map[in_name]["signed"] = True
                    in_map[in_name]["arr"] = False
                    in_map[in_name]["n_elem"] = 1
                elif "uint8" in value and "[" in value:
                    in_map[in_name]["type"] = "int"
                    in_map[in_name]["n_bits"] = 8
                    in_map[in_name]["signed"] = False
                    in_map[in_name]["arr"] = True
                    n_elem = int(value.replace('uint8', '').replace('[', '').replace(']', ''))
                    in_map[in_name]["n_elem"] = n_elem
                elif "uint64" in value and "[" in value:
                    in_map[in_name]["type"] = "int"
                    in_map[in_name]["n_bits"] = 64
                    in_map[in_name]["signed"] = False
                    in_map[in_name]["arr"] = True
                    n_elem = int(value.replace('uint64', '').replace('[', '').replace(']', ''))
                    in_map[in_name]["n_elem"] = n_elem
                elif "int32" in value and "[" in value:
                    in_map[in_name]["type"] = "int"
                    in_map[in_name]["n_bits"] = 32
                    in_map[in_name]["signed"] = True
                    in_map[in_name]["arr"] = True
                    n_elem = int(value.replace('int32', '').replace('[', '').replace(']', ''))
                    in_map[in_name]["n_elem"] = n_elem
                elif "float32" in value and "[" in value:
                    in_map[in_name]["type"] = "float"
                    in_map[in_name]["n_bits"] = 32
                    in_map[in_name]["signed"] = True
                    in_map[in_name]["arr"] = True
                    n_elem = int(value.replace('float32', '').replace('[', '').replace(']', ''))
                    in_map[in_name]["n_elem"] = n_elem
                elif "float64" in value and "[" in value:
                    in_map[in_name]["type"] = "float"
                    in_map[in_name]["n_bits"] = 64
                    in_map[in_name]["signed"] = True
                    in_map[in_name]["arr"] = True
                    n_elem = int(value.replace('float64', '').replace('[', '').replace(']', ''))
                    in_map[in_name]["n_elem"] = n_elem
                else:
                    print("\n[Forest]: Error! Config file error. Input type not recognized at signal {}\n".format(in_name))
                    sys.exit(1)
            elif ("Address" in key and reading_input):
                if in_map[in_name]["protocol"] == "lite":
                    in_map[in_name]["addr"] = int(value)
                else:
                    in_map[in_name]["addr"] = None

            # Outputs
            elif (key == "Output name"):
                out_map[value] = {}
                reading_input = False
                reading_output = True
                out_name = value
            elif (key == "Protocol" and reading_output):
                out_map[out_name]["protocol"] = value.lower()
                if "stream" in value.lower():
                    has_axis_out = True
            elif (key == "Type" and reading_output):
                if "[" not in value or "]" not in value:
                    print("\n[Forest]: Error! Config file error. All output signals must be fixed size arrays at signal {}\n".format(out_name))
                    sys.exit(1)
                if "uint8" in value and "[" in value:
                    out_map[out_name]["type"] = "int"
                    out_map[out_name]["n_bits"] = 8
                    out_map[out_name]["signed"] = False
                    out_map[out_name]["arr"] = True
                    n_elem = int(value.replace('uint8', '').replace('[', '').replace(']', ''))
                    out_map[out_name]["n_elem"] = n_elem
                elif "uint64" in value and "[" in value:
                    out_map[out_name]["type"] = "int"
                    out_map[out_name]["n_bits"] = 64
                    out_map[out_name]["signed"] = False
                    out_map[out_name]["arr"] = True
                    n_elem = int(value.replace('uint64', '').replace('[', '').replace(']', ''))
                    out_map[out_name]["n_elem"] = n_elem
                elif "int32" in value and "[" in value:
                    out_map[out_name]["type"] = "int"
                    out_map[out_name]["n_bits"] = 32
                    out_map[out_name]["signed"] = True
                    out_map[out_name]["arr"] = True
                    n_elem = int(value.replace('int32', '').replace('[', '').replace(']', ''))
                    out_map[out_name]["n_elem"] = n_elem
                elif "float32" in value and "[" in value:
                    out_map[out_name]["type"] = "float"
                    out_map[out_name]["n_bits"] = 32
                    out_map[out_name]["signed"] = True
                    out_map[out_name]["arr"] = True
                    n_elem = int(value.replace('float32', '').replace('[', '').replace(']', ''))
                    out_map[out_name]["n_elem"] = n_elem
                elif "float64" in value and "[" in value:
                    out_map[out_name]["type"] = "float"
                    out_map[out_name]["n_bits"] = 64
                    out_map[out_name]["signed"] = True
                    out_map[out_name]["arr"] = True
                    n_elem = int(value.replace('float64', '').replace('[', '').replace(']', ''))
                    out_map[out_name]["n_elem"] = n_elem
                else:
                    print("\n[Forest]: Error! Config file error. Output type not recognized at signal {}\n".format(out_name))
                    sys.exit(1)
            elif ("Address" in key and reading_output):
                if out_map[out_name]["protocol"] == "lite":
                    out_map[out_name]["addr"] = int(value)
                else:
                    out_map[out_name]["addr"] = None
    f.close()

    check_in_map_validity(in_map)

    check_out_map_validity(out_map)

    print("\n[Forest]: Starting the tool...\n")
 
    if not os.path.exists('output'):
        os.makedirs('output')

    prj = "forest_" + prj
 
    in_signals = in_map.keys()

    out_signals = out_map.keys()

    qos = 10

    if (not dev_ws.endswith('/')):
        dev_ws += '/'

    # Rendering of template files

    print("\n[Forest]: Rendering templates according to user inputs...\n")

    # Render interface package.xml

    render_int_package_xml(env, prj)

    # Render interface CMakeLists.txt

    render_int_cmakelists_txt(env, prj)

    # Render node package.xml

    render_node_package_xml(env, prj)

    # Render node setup.py

    render_node_setup_py(env, prj, test_enabled)

    # Render node ros_fpga_lib.py

    render_node_ros_fpga_lib_py(env, bit_file, ip_name, in_map, out_map, has_axis_in, has_axis_out)

    # Render node fpga_node.py

    render_node_fpga_node_py(env, prj, qos, in_signals, out_signals, has_axis_in, has_axis_out)

    # Render talker and listener if the test option is enabled

    if test_enabled:
        render_test_talker(env, prj, qos, in_map)
        render_test_listener(env, prj, qos)

    # Generate message package
    
    create_msg_pkg(dev_ws, prj, in_map, out_map)

    # Generate node package

    create_fpga_node_pkg(dev_ws, prj, test_enabled)

    return 0

if __name__ == '__main__':
    main()


