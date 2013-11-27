target = "xilinx"
action = "synthesis"

syn_device = "xc3s1500"
syn_grade = "-4"
syn_package = "fg456"
syn_top = "topdpram"
syn_project = "cvora.xise"

files = ["../sources/topdpram.ucf"]

modules = { "local" : "../sources" }
