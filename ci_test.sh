#!/bin/bash -ex
# author ShanakaPrageeth
# details executing Github Actions CI/CD pipeline

DEBIAN_FRONTEND=noninteractive
PROGRAM_NAME="$(basename $0)"
BASEDIR=$(dirname $(realpath "$0"))

export PYTHONPATH=$BASEDIR:$PYTHONPATH

install_dependencies(){
    cd $BASEDIR/
    sudo apt-get install -y python3 python3-pip python3-venv npm
    python3 -m venv .venv
    source .venv/bin/activate
    python3 -m pip install --upgrade pip
    python3 -m pip install -r requirements.txt
    sudo npm install --unsafe-perm node-red
    sudo npm install -g --unsafe-perm node-red
    sudo npm install node-red-dashboard
    sudo npm install -g node-red-dashboard
}

nodered(){
    source .venv/bin/activate
    if netstat -tulpn | grep ':1880' > /dev/null
    then
        echo "Node-RED is already running on port 1880."
        cd m5stack-shana/digitaltwin-demo/node_red_dashboard/
        curl -X POST http://localhost:1880/flows -H "Content-Type: application/json" --data "@node_red_digital_twin.json"
    else
        echo "Node-RED is not running on port 1880. Starting a new instance..."
        node-red -p 1880  --json $BASEDIR/m5stack-shana/digitaltwin-demo/node_red_dashboard/node_red_digital_twin.json
        cd $BASEDIR/m5stack-shana/digitaltwin-demo/node_red_dashboard/
        curl -X POST http://localhost:1880/flows -H "Content-Type: application/json" --data "@node_red_digital_twin.json"
    fi
}

setup_heater_control(){
    cd $BASEDIR/m5stack-shana/digitaltwin-demo/actuator
    python3 heater.py --config $BASEDIR/m5stack-shana/digitaltwin-demo/config.yml
}

setup_sim(){
    source $BASEDIR/.venv/bin/activate
    cd $BASEDIR/m5stack-shana/digitaltwin-demo/simulation
    python3 sim_greenhouse.py --config $BASEDIR/m5stack-shana/digitaltwin-demo/config.yml
}

setup_local_server(){
    install_dependencies
    sudo apt-get install -y mosquitto mosquitto-clients git build-essential terminator screen curl net-tools
    sudo cp $BASEDIR/mosquitto.conf /etc/mosquitto/conf.d/
    sudo systemctl enable mosquitto
    sudo systemctl start mosquitto
    sudo systemctl status mosquitto
}

unittests() {
    echo "Executing tox tests"
    tox
}

systemtests(){
    cd $BASEDIR/examples
    echo "======Running System Tests====="
    echo "Running Config Reader Test"
    python3 config_reader_example.py $BASEDIR/config.yml | tee output.log
    if ! grep -q "Configuration loaded successfully" output.log; then
        echo "Error: Configuration failed to load"
        exit 1
    else
        echo "Test Passed"
    fi
    exit 0
    echo "Running MQTT Node Test"
    # TODO move password to secret
    python3 mqtt_test_example.py \
            --mqtt_sub_topic test \
            --mqtt_pub_topic test \
            --mqtt_port 8883 \
            --mqtt_broker 3277bbbd24e24f328e93456f6a9f1602.s1.eu.hivemq.cloud \
            --mqtt_username ci-runner \
            --mqtt_password cirunner1A \
            --enable_tls --iterations 2 | tee output.log

    cat output.log

    if ! grep -q "Received Client" output.log; then
        echo "Error: Subscription message not received"
        exit 1
    else
        echo "Test Passed"
        exit 0
    fi
}

if [[ $# -eq 0 ]]; then
    echo "No argument provided. Defaulting to 'all'. Options are: unittests, systemtests, all"
    set -- all
fi

case "$1" in
    unittests)
        unittests
        ;;
    systemtests)
        systemtests
        ;;
    install_dependencies)
        install_dependencies
        ;;
    nodered)
        nodered
        ;;
    setup_local_server)
        setup_local_server
        ;;
    setup_heater_control)
        setup_heater_control
        ;;
    setup_sim)
        setup_sim
        ;;
    all)
        unittests
        systemtests
        ;;
    *)
        echo "Invalid argument: $1"
        echo "Usage: $PROGRAM_NAME {unittests|systemtests|install_dependencies|all|nodered|setup_local_server|setup_heater_control|setup_sim}"
        exit 1
        ;;
esac


# useful commands
# export PYTHONPATH=$(pwd):$PYTHONPATH
# mosquitto_sub -h 127.0.0.1 -p 1883 -t _1451DT/#
# mosquitto_sub -h 127.0.0.1 -p 1883 -t _1451DT/core_1/sensor/data
# mosquitto_pub -h 127.0.0.1 -p 1883 -t _1451DT/core_1/sensor/data -m helloworld
# mosquitto_sub -h 127.0.0.1 -p 1883 -t _1451DT/core_2/sensor/data
# mosquitto_pub -h 127.0.0.1 -p 1883 -t _1451DT/core_2/sensor/data -m helloworld
# node-red -p 1880 --json node_red_digital_twin.json
# deploy automatically
# curl -X POST http://localhost:1880/flows -H "Content-Type: application/json" --data "@node_red_digital_twin.json"
