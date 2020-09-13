#!/bin/bash
cd dynopt_code/tbxmanager
matlab -nodisplay -nodesktop -r "run tbxmanager; run savepath; tbxmanager restorepath; exit"
cd ../..
