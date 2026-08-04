#!/bin/bash
# auto_stub.sh
while true; do
    OUTPUT=$(g++ -shared -fPIC -I. -Istubs -o kinematics.so \
        src/Kinematics/RotatingTableFiveAxis.cpp 2>&1)
    
    # Extract missing header from error
    MISSING=$(echo "$OUTPUT" | grep "fatal error:" | \
        sed "s/.*fatal error: \(.*\): No such file.*/\1/")
    
    if [ -z "$MISSING" ]; then
        echo "Done! Either compiled or hit a different error."
        echo "$OUTPUT"
        break
    fi

    echo "Stubbing: $MISSING"
    mkdir -p "stubs/$(dirname $MISSING)"
    echo "#pragma once" > "stubs/$MISSING"
done
