import { Robot, WheelOperation, ShoulderOperation, Operation, ServoOperation, DrumOperation } from '../../src/enumerations/robot-commands';
import HTTP from '../../src/api/web-commands';

const DEFAULT_EXT = '/';
let currentCommandID = 0; // Flag for command number


export default class EZRASSOR {

    constructor(host, route = '') {
        this.host = host;
        this.route = route;
        this.setCoordinate(0, 0);
        this.allStop();
    }

    // Getters and Setters
    get host() {
        return this._host;
    }

    set host(value) {
        this._host = value;
    }

    get route() {
        return this._route;
    }

    set route(value) {
        if (value[0] === '/') {
            this._route = value.substring(1);
            return;
        }

        this._route = value;
    }

    get coordinate() {
        return this._coordinate;
    }

    setCoordinate(combinedCoordinate) {

        var coordinates = (typeof combinedCoordinate === "string") ? combinedCoordinate.trim() : '0';
        var split = coordinates.split(',');

        var x = split[0];
        var y = '0';

        if (split.length == 2 && split[1] != '') {
            y = split[1];
        }

        this._coordinate = {
            x: parseInt(x),
            y: parseInt(y)
        }
    }
    
    formatIP(host) {
        const ip =  host.split(":")[0];
        if (!ip) {
            throw new Error("IP address cannot be empty");
        }
        return "ip_" + ip.replace(/\./g, "_");
    }

    // Build complete apiPath for HTTP requests
    get apiPath() {
        console.log('http://' + this.host + '/' + this.route);
        return this.host + this.route;
    }

    // Return custom twist message
    get twistMsg() {
        return JSON.stringify(this._twistMsg);
    }

    get toggleDetectionMsg() {
        return JSON.stringify(this.toggleDetectionMsg);
    }

    // Update only the instruction needed
    updateTwistMsg(instruction) {
        this._twistMsg = instruction;
    }
    updateDrumTwistMsg(operation)
    {
        var instruction = null;
        switch (operation) {
            case DrumOperation.DIG:
                instruction = {front_drum_action: {linear_x: -1}}
                break;
            case DrumOperation.DUMP:
                instruction = {front_drum_action: {linear_x: 1}}
                break;
            case DrumOperation.STOP:
                instruction = {front_drum_action: {linear_x: 0}}
                break;
            default:
                console.log('Invalid drum part selected');
                console.log(operation);
                console.log(DrumOperation.DUMP);
                return;
        }
        this._twistMsg = instruction;
    }
    updateShoulderTwistMsg(isFront, operation){
        var instruction = null;
        switch (operation) {
            case ShoulderOperation.RAISE:
                if(isFront)
                    instruction = { shoulder_action: { linear_y: ShoulderOperation.LINEAR_Y_UP, angular_y: 0, linear_x: 0, angular_z: 0 } }
                else
                    instruction = { shoulder_action: { linear_y: 0, angular_y: ShoulderOperation.ANGULAR_Y_UP, linear_x: 0, angular_z: 0 } }
                break;
            case ShoulderOperation.LOWER:
                if(isFront)
                    instruction = { shoulder_action: { linear_y: ShoulderOperation.LINEAR_Y_DOWN, angular_y: 0, linear_x: 0, angular_z: 0  } }
                else
                    instruction = { shoulder_action: { linear_y: 0, angular_y: ShoulderOperation.ANGULAR_Y_DOWN, linear_x: 0, angular_z: 0  } }
                break;
            case ShoulderOperation.STOP:
                instruction = { shoulder_action: { linear_y: 0, angular_y: 0, linear_x: 0, angular_z: 0 } }
                break;
            default:
                console.log('Invalid Shoulder part selected');
                return;
        }
        this._twistMsg = instruction;
    }

    updateWheelTwistMsg(operation) {
        var instruction = null;
        switch (operation) {
            case WheelOperation.FORWARD:
                instruction = { wheel_action: { linear_x: WheelOperation.LINEAR_X_FORWARD, angular_z: 0 } }
                break;
            case WheelOperation.BACKWARD:
                instruction = { wheel_action: { linear_x: WheelOperation.LINEAR_X_BACKWARD, angular_z: 0 } }
                break;
            case WheelOperation.LEFT:
                instruction = { wheel_action: { linear_x: 0, angular_z: WheelOperation.ANGULAR_Z_LEFT } }
                break;
            case WheelOperation.RIGHT:
                instruction = { wheel_action: { linear_x: 0, angular_z: WheelOperation.ANGULAR_Z_RIGHT } }
                break;
            case WheelOperation.STOP:
                instruction = { wheel_action: { linear_x: 0, angular_z: 0 } }
                break;
            default:
                console.log('Invalid wheel part selected');
                return;
        }
        this._twistMsg = instruction;
    }

    updateAutonomyTwistMsg(instruction) {
        if (instruction == Operation.DRIVE || instruction == Operation.FULLAUTONOMY) {
            this._twistMsg = {
                autonomous_toggles: instruction,
                target_coordinate: this.coordinate
            }
            return;
        }

        this._twistMsg = { autonomous_toggles: instruction };
    }

    // Stop all robot operations
    allStop = () => {
        this._twistMsg = {
            autonomous_toggles: 0,
            target_coordinate: this.coordinate,
            wheel_instruction: "none",
            front_arm_instruction: 0,
            back_arm_instruction: 0,
            front_drum_instruction: 0,
            back_drum_instruction: 0
        }

        HTTP.doPost(this.apiPath, this.twistMsg, DEFAULT_EXT);
    }

    // Sends the command and returns 1 for success, 0 for failed, -1 for cancel, 404 for error
    async sendCommand() {
        let formattedIP = this.formatIP(this.host);
        currentCommandID++; // Increment commandID each time a new command is sent
        const commandID = currentCommandID; // Store local copy to track if it changes
    
        await HTTP.doPost(this.apiPath, this.twistMsg, DEFAULT_EXT);
        // let response = await HTTP.doGet(this.apiPath, `/${formattedIP}/command_status`);
        await new Promise(resolve => setTimeout(resolve, 200)); // 200ms delay
        let response = await HTTP.doGet(this.apiPath, `/${formattedIP}/command_status`);

        // Extract the status (removes the trailing numbers)
        let status = response.status.replace(/\d+$/, '');
        
        // Extract the number (finds the trailing numbers and converts to integer)
        let commandNumber = parseInt(response.status.match(/\d+$/)[0], 10);
        
        // console.log("Status:", status, "Command #:", commandNumber);
       
        let attemptsMade = 1;
    
        if (status == "Success")
            return 1;
        while (status === "Timeout") {
            await new Promise(resolve => setTimeout(resolve, 1000)); // 1-second delay

            // Overwrite command was sent, end early
            if (commandID !== currentCommandID) {
                console.log("New command detected, stopping retries.");
                return -1;
            }
    
            // Failed all attemps
            if (attemptsMade > 2) {
                console.log("Failed to send the command...");
                return 0;
            }
    
            console.log(`Attempting to resend command, attempt ${attemptsMade}/3`);
            await HTTP.doPost(this.apiPath, this.twistMsg, DEFAULT_EXT);
            await new Promise(resolve => setTimeout(resolve, 200)); // 200ms delay
            response = await HTTP.doGet(this.apiPath, `/${formattedIP}/command_status`);
            status = response.status.replace(/\d+$/, '');
            commandNumber = parseInt(response.status.match(/\d+$/)[0], 10);
            
            // console.log("Status:", status, "Command #:", commandNumber);
            attemptsMade++;
    
            // Successful command
            if(status === "Success")
                return 1;
        }
        return 404; // we should never really end up here unless something is fundamentally wrong
    }


    // Execute the corresponding robot command from the enumeration items passed in
    async executeRobotCommand(part, operation) {
        // Needed when a stop override needs to occur
        if (part == Robot.ALL && operation == Operation.STOP) {
            this.allStop();
            return;
        }

        switch (part) {
            case Robot.FRONTARM:
                this.updateShoulderTwistMsg(true, operation);
                //this.updateTwistMsg({ front_arm_action: operation });
                break;
            case Robot.BACKARM:
                this.updateShoulderTwistMsg(false, operation);
                //this.updateTwistMsg({ back_arm_action: operation });
                break;
            case Robot.FRONTDRUM:
                this.updateDrumTwistMsg(operation);
                break;
            case Robot.BACKDRUM:
                this.updateTwistMsg({ back_drum_action: operation });
                break;
            case Robot.WHEELS:
                // this.updateWheelTwistMsg({wheel_action:operation});
                this.updateWheelTwistMsg(operation);

                break;
            case Robot.AUTONOMY:
                this.updateAutonomyTwistMsg(routine_action);
                break;
			case Robot.SERVO:
            default:
                console.log('Invalid robot part selected');
                return;
        }
        try {
            let commandStatus = await this.sendCommand(); 
            return commandStatus; 
        } catch (error) {
            console.log("Command Status Error: " + error);
            return false;  // Return false in case of error
        }
        
    }
}