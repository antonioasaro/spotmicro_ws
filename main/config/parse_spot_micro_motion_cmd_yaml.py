# Parse yaml params files into .h file
import re

count = 0
rd_filename = "spot_micro_motion_cmd.yaml"
wr_filename = "spot_micro_motion_cmd_params.h"
wr_f = open(wr_filename, "w")
with open(rd_filename, "r") as rd_f:
    for line in rd_f:
        if not line.strip():
            continue
        else:
            if (line.startswith(" ") or
                    line.startswith("standalone") or
                    line.startswith("RF_") or
                    line.startswith("RB_") or
                    line.startswith("LB_") or
                    line.startswith("LF_") or
                    line.startswith("rb_contact_phases") or
                    line.startswith("rf_contact_phases") or
                    line.startswith("lf_contact_phases") or
                    line.startswith("lb_contact_phases") or
                    line.startswith("body_shift_phases")
                    ):
                continue
            else:
                if not line.startswith("#"):
                    count = count + 1
                    newline = line.rstrip('\r\n')
                    newline = re.sub("#.*", "", newline)
                    newline = newline.replace("foot_x", "feet_x")
                    newline = newline.replace(":", " =")
                    if (count <= 5):
                        newline = "smnc_.smc." + newline + ";"
                    else:
                        newline = "smnc_." + newline + ";"
                    newline = re.sub(" *;", ";", newline)
                    wr_f.write(newline + "\n")


#### RF_3: {num: 1,  center: 306,range: 372,direction:  1, center_angle_deg:  88.2}
with open(rd_filename, "r") as rd_f:
    for line in rd_f:
        if (line.startswith("RF_") or
                line.startswith("RB_") or
                line.startswith("LB_") or
                line.startswith("LF_")
            ):
            li = line.rstrip('\r\n')
            li = li.split(":", 1)
            li[1] = re.sub(" {", "{", li[1])
            li[1] = re.sub(",", "},", li[1])
            li[1] = re.sub("num: ", "\"num\", ", li[1])
            li[1] = re.sub("center: ", "{\"center\", ", li[1])
            li[1] = re.sub("range: ", " {\"range\", ", li[1])
            li[1] = re.sub("direction: ", " {\"direction\", ", li[1])
            li[1] = re.sub("center_angle_deg: ",
                           " {\"center_angle_deg\", ", li[1])
            wr_f.write("smnc_.servo_config[\"" + li[0] + "\"] = {" + li[1] + "};\n")


#### rb_contact_phases: [1, 0, 1, 1, 1, 1, 1, 1]
with open(rd_filename, "r") as rd_f:
    for line in rd_f:
        if (line.startswith("rb_contact_phases: ") or
                line.startswith("rf_contact_phases: ") or
                line.startswith("lf_contact_phases: ") or
                line.startswith("lb_contact_phases: ") or
                line.startswith("body_shift_phases: ")
            ):
            li = line.rstrip('\r\n')
            li = re.sub(" ", "", li);
            li = li.split(":", 1)
            li[1] = re.sub("\[", "", li[1])
            li[1] = re.sub("\]", "", li[1])
            ph = li[1].split(",")
            for x in ph:
                wr_f.write("smnc_." + li[0] + ".push_back(" + x + ");\n")

wr_f.close()
