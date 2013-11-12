target = "xilinx"
action = "synthesis"

syn_device = "xc3s1500"
syn_grade = "-4"
syn_package = "fg456"
syn_top = "top_cvora"
syn_project = "cvora.xise"

files = ["../sources/top_cvora.ucf"]

modules = { "local" : ["../sources"],
            "git"   : ["git://ohwr.org/hdl-core-lib/general-cores.git::proposed_master"]}

fetchto="../ip_cores"
