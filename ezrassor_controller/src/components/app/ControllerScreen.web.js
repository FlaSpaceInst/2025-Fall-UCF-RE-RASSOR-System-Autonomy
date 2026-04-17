import React from 'react';
import Modal from 'react-native-modal';
import InformationController from './InformationController';
import DebugController from './DebugController';
import EZRASSOR from '../../../src/api/ezrassor-service';
import ControllerStyle from '../../../src/styles/controller';
import { Robot, ShoulderOperation, DrumOperation, WheelOperation, Operation } from '../../../src/enumerations/robot-commands';
import {
    Text,
    View,
    Pressable,
    Image,
    StatusBar,
    KeyboardAvoidingView,
    TextInput,
} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font from 'expo-font';
import { isIpReachable } from '../../functionality/connection';
import AsyncStorage from '@react-native-async-storage/async-storage';

const CONNECTION_POLLING_INTERVAL = 2000;
const CONNECTION_POLLING_TIMEOUT = 4000;
const CONNECTION_POLLING_VERBOSE = false;

/**
 * React component for the actual rover controller interface.
 */
export default class ControllerScreen extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            autonomyModalVisible: false,
            infoModalVisible: false,
            devModalVisible: false,
            debugModalVisible: false,
            ls: 0,
            rs: 0,
            ipModal: false,
            xyModal: false,
            isLoading: true,
            syncShoulders: false,
            syncDrums: false,
            toggleLeftDig: false,
            toggleRightDig: false,
            toggleLeftDump: false,
            toggleRightDump: false,
            toggleSyncDig: false,
            toggleSyncDump: false,
            control: 0,
            colorValue: { r: 58, g: 61, b: 61 }, // Grey color
            circleColorValue: { r: 91, g: 91, b: 91 }, // Light color
            xy: '0,0',
            ip: '',
        };
        this.interval = null; // To hold the interval ID
        this.circleInterval = null;

        this.EZRASSOR = new EZRASSOR(this.state.ip);
    }

    async componentDidMount() {
        await this.getIpFromStorage();

        this.setState({ isLoading: false, ls: this.props.route.params?.currentls || 0,
			rs: this.props.route.params?.currentrs || 0, });

        this._unsubscribe = this.props.navigation.addListener('focus', async () => {
            this.getIpFromStorage();
        });

        // Set up the connection polling logic.
        let pollCount = 0;

        const pollInterval = (CONNECTION_POLLING_INTERVAL / 1000.0).toFixed(2);
        const pollTimeout = (CONNECTION_POLLING_TIMEOUT / 1000.0).toFixed(2);
        console.log(`Polling every ${pollInterval}s with a ${pollTimeout}s timeout.`);

    }

    async componentWillUnmount() {
        clearInterval(this.connectionPoller);
        clearInterval(this.interval);
        this._unsubscribe();
    }

    async getIpFromStorage() {
        try {
            const ip = await AsyncStorage.getItem('myIp');

            if (ip != null) {
                this.changeIP(ip);
            }
        } catch (error) {
            // Error retrieving data. Do nothing.
        }
    }

    setAutonomyModalVisible(visible) {
        this.setState({ autonomyModalVisible: visible });
    }

    setInfoModalVisible(visible) {
        this.setState({ infoModalVisible: visible });
    }

    setDebugModalVisible(visible) {
        this.setState({ debugModalVisible: visible});
    }


    setDevModalVisible(visible) {
        this.setState({ devModalVisible: visible });
    }

    setIPModalVisible(visible) {
        this.setState({ ipModal: visible });
    }

    setXYModalVisible(visible) {
        this.setState({ xyModal: visible });
    }

    changeXY(combined) {
        this.setState({ xy: combined }, () => {
            this.EZRASSOR.setCoordinate(this.state.xy);
        });
    }

    /**
     * Set `this.state.ip` and `this.EZRASSOR.host` to the specified IP + port.
     *
     * @param {string} ip IP + port.
     */
    changeIP(ip) {
        this.setState({ ip }, () => {
            this.EZRASSOR.host = ip;
        });
    }

    /**
     * Update animation frame before processing click so that opacity can change on click.
     */
    sendOperation(part, operation) {
        // Stop loading animation if we swap from wheel to drum/shoulders
        if (part !== Robot.WHEELS) {
            this.stopFlashing(); // Stop the flashing animation
        }
        
        return new Promise((resolve, reject) => {
            requestAnimationFrame(() => {
                let commandStatus = this.EZRASSOR.executeRobotCommand(part, operation);
                commandStatus.then(result => {
                    resolve(result);  // Resolve the Promise with the result
                }).catch(error => {
                    reject(error);  // Reject the Promise on error
                });
            });
        });
    }


    fadeButton = () => {
        this.interval = setInterval(() => {
            this.setState((prevState) => {
                const { r, g, b } = prevState.colorValue;
                // Calculate new color values to fade to grey (#3a3d3d or RGB(58, 61, 61))
                const newR = r > 58 ? r - 1 : 58;
                const newG = g < 61 ? g + 1 : 61;
                const newB = b < 61 ? b + 1 : 61;
    
                // Check if we've reached grey
                if (newR === 58 && newG === 61 && newB === 61) {
                    clearInterval(this.interval); // Clear interval when we reach grey
                }
    
                return {
                    colorValue: { r: newR, g: newG, b: newB },
                };
            });
        }, 25); // Change the interval timing to adjust the fade speed
    }
    


    // Makes the circles flash when receiving a command
    startFlashing = () => {
        if (this.circleInterval) 
            clearInterval(this.circleInterval); // Clear existing interval before starting a new one
        let greyToWhite = true;
        let glowValue = 0;
        this.circleInterval = setInterval(() => {
            this.setState((prevState) => {
                let newR,newG,newB;
                const { r, g, b } = prevState.circleColorValue;
                if(greyToWhite)
                {
                    newR = r < 255 ? r + 5 : 255;
                    newG = g < 255 ? g + 5 : 255;
                    newB = b < 255 ? b + 5 : 255;
                    glowValue += 0.1
                    glowValue *= 1.2
                }
                else
                {
                    newR = r > 91 ? r - 5 : 91;
                    newG = g > 91 ? g - 5 : 91;
                    newB = b > 91 ? b - 5 : 91;

                }
                if(r == 255 && g == 255 && b == 255)
                    greyToWhite = false;
                if(r == 91 && g == 91 && b == 91)
                {
                    greyToWhite = true;
                    glowValue = 0
                }

                // Toggle between white (255, 255, 255) and grey (91, 91, 91)

                return {
                    circleColorValue: { r: newR, g: newG, b: newB },
                    glowIntensity: glowValue, // 0 when dark, 10 when white
                };
            });
        }, 25); // Adjust timing as needed
    };

    stopFlashing = () => {
        if (this.circleInterval) {
            clearInterval(this.circleInterval);
            this.circleInterval = null; // Reset the interval reference
        }
        this.setState({
            circleColorValue: { r: 91, g: 91, b: 91 }, // Reset to dark grey
            glowIntensity: 0, // Reset glow
        });
    };


    render() {
        const { ls, rs } = this.state;
        // I.e., don't do full render if font is still loading...
        if (this.state.isLoading) {
            return <View style={{ flex: 1, backgroundColor: '#5D6061' }} />;
        }

        return (
            <View style={ControllerStyle.container}>
                <StatusBar hidden />

                {/* Autonomy popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.autonomyModalVisible}
                    onSwipeComplete={() => this.setAutonomyModalVisible(!this.state.autonomyModalVisible)}
                    swipeDirection={["down", "up", "left", "right"]}
                    onRequestClose={() =>
                        this.setAutonomyModalVisible(!this.state.autonomyModalVisible)
                    }
                >
                    <Pressable style={{ flex: 1, marginHorizontal: 15, justifyContent: "center" }}>
                        <View>

                            {/* Modal title container. */}
                            <View style={{ flexDirection: "row", marginVertical: 15, justifyContent: "center" }}>
                                <Text style={ControllerStyle.textLarge}>
                                    Activate Autonomous Function(s)
                                </Text>
                            </View>

                            {/* Modal body container. */}
                            <View style={{ flexDirection: "row", justifyContent: 'space-evenly' }}>

                                {/* Button: Drive. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => this.setXYModalVisible(true)}
                                >
                                    <Text style={ControllerStyle.textSmall}>Drive</Text>
                                </Pressable>

                                {/* Button: Dig. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.DIG);
                                    }}
                                >
                                    <Text style={ControllerStyle.textSmall}>Dig</Text>
                                </Pressable>

                                {/* Button: Dump. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.DUMP);
                                    }}
                                >
                                    <Text style={ControllerStyle.textSmall}>Dump</Text>
                                </Pressable>

                                {/* Button: Self-Right. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.SELFRIGHT);
                                    }}
                                >
                                    <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>
                                        Self - Right
                                    </Text>
                                </Pressable>

                                {/* Button: Full-Auto. */}
                                <Pressable
                                    style={ControllerStyle.modalButton}
                                    onPress={() => {
                                        this.sendOperation(Robot.AUTONOMY, Operation.FULLAUTONOMY);
                                    }}
                                >
                                    <Text style={[ControllerStyle.textSmall, ControllerStyle.columnText]}>
                                        Auto Mode
                                    </Text>
                                </Pressable>

                            </View>

                        </View>
                    </Pressable>

                </Modal>

                {/* Info popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.infoModalVisible}
                    onSwipeComplete={() => this.setInfoModalVisible(false)}
                    swipeDirection={["down", "up", "left", "right"]}
                    onRequestClose={() => this.setInfoModalVisible(false)}
                >
					{/* Modal X button. */}
                <Pressable
                style={[ControllerStyle.buttonModalContainer]}
                onPress={() => {
                    this.setInfoModalVisible(false);
                }}
                >
				<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="36" height="36">
					<line x1="18" y1="6" x2="6" y2="18" stroke="black" strokeWidth="2" strokeLinecap="round"/>
					<line x1="6" y1="6" x2="18" y2="18" stroke="black" strokeWidth="2" strokeLinecap="round"/>
				</svg>
                </Pressable>
                    <InformationController></InformationController>
                </Modal>

                {/* Info popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.debugModalVisible}
                    onRequestClose={() => this.setDebugModalVisible(false)}
                >
                        {/* Modal X button. */}
                    <Pressable
                    style={[ControllerStyle.buttonModalContainer]}
                    onPress={() => {
                        this.setDebugModalVisible(false);
                    }}
                    >
                    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="36" height="36">
                        <line x1="18" y1="6" x2="6" y2="18" stroke="black" strokeWidth="2" strokeLinecap="round"/>
                        <line x1="6" y1="6" x2="18" y2="18" stroke="black" strokeWidth="2" strokeLinecap="round"/>
                    </svg>
                    </Pressable>
                    <DebugController></DebugController>
                </Modal>

                {/* Drive-autonomy-input modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.xyModal}
                    onSwipeComplete={() => this.setXYModalVisible(false)}
                    swipeDirection="down"
                    onRequestClose={() => {
                        this.setXYModalVisible(false);
                    }}
                >
                    <KeyboardAvoidingView paddingLeft={64} paddingRight={64}>

                        {/* Prompt. */}
                        <Text
                            style={[ControllerStyle.textSmall, ControllerStyle.columnText]}
                        >
                            Enter the X,Y coordinates where the robot will drive to
                        </Text>

                        {/* Input field. */}
                        <TextInput
                            style={ControllerStyle.ipInputBox}
                            onChangeText={(text) => this.changeXY(text)}
                            value={this.state.xy}
                            placeholder="x,y"
                            marginVertical={20}
                        />

                        {/* Done button. */}
                        <Pressable
                            style={{
                                alignItems: 'center',
                                backgroundColor: '#DDDDDD',
                                padding: 10,
                            }}
                            onPress={() => {
                                this.sendOperation(Robot.AUTONOMY, Operation.DRIVE);
                                this.setXYModalVisible(false);
                            }}
                        >
                            <Text>Done</Text>
                        </Pressable>

                    </KeyboardAvoidingView>
                </Modal>

                {/* Controller screen top row controls. */}
                <View style={ControllerStyle.headerContainer}>
                    <View style={ControllerStyle.evenlySpaceIcons}>
                        {/* Info button. */}
                        <Pressable
                            style={[ControllerStyle.icons, {outline: 'none'}]}
                            onPress={() => { this.setInfoModalVisible(true); }}
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="50%" width="50%">
								<path fill="#ffffff" d="M256 512A256 256 0 1 0 256 0a256 256 0 1 0 0 512zM216 336l24 0 0-64-24 0c-13.3 0-24-10.7-24-24s10.7-24 24-24l48 0c13.3 0 24 10.7 24 24l0 88 8 0c13.3 0 24 10.7 24 24s-10.7 24-24 24l-80 0c-13.3 0-24-10.7-24-24s10.7-24 24-24zm40-208a32 32 0 1 1 0 64 32 32 0 1 1 0-64z"/>
							</svg>
                        </Pressable>

                        {/* Set-IP button. Wifi Icon*/}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("IPConnect Screen")}
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" height="50%" width="50%">
								<path fill="#ffffff" d="M54.2 202.9C123.2 136.7 216.8 96 320 96s196.8 40.7 265.8 106.9c12.8 12.2 33 11.8 45.2-.9s11.8-33-.9-45.2C549.7 79.5 440.4 32 320 32S90.3 79.5 9.8 156.7C-2.9 169-3.3 189.2 8.9 202s32.5 13.2 45.2 .9zM320 256c56.8 0 108.6 21.1 148.2 56c13.3 11.7 33.5 10.4 45.2-2.8s10.4-33.5-2.8-45.2C459.8 219.2 393 192 320 192s-139.8 27.2-190.5 72c-13.3 11.7-14.5 31.9-2.8 45.2s31.9 14.5 45.2 2.8c39.5-34.9 91.3-56 148.2-56zm64 160a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z"/>
							</svg>
                        </Pressable>

                        {/* <Pressable
                            style={[ControllerStyle.icons, {outline: 'none', fill: 'white', paddingLeft: 0}]}
                            onPress={() => { this.setDebugModalVisible(true); }}
                        >
                            <svg xmlns="http://www.w3. rg/2000/svg" x="0px" y="0px" width="65%" height="65%" viewBox="0 0 32 32">
                                <path fill="#fffff" d="M 2 7 L 2 25 L 30 25 L 30 7 L 2 7 z M 4 9 L 28 9 L 28 23 L 4 23 L 4 9 z M 6 11 L 6 21 L 9 21 C 10.654 21 12 19.654 12 18 L 12 14 C 12 12.346 10.654 11 9 11 L 6 11 z M 16 11 C 14.897 11 14 11.897 14 13 L 14 19 C 14 20.103 14.897 21 16 21 L 18 21 L 18 19 L 16 19 L 16 17 L 18 17 L 18 15 L 16 15 L 16 13 L 18 13 L 18 11 L 16 11 z M 19.691406 11 L 21.775391 20.025391 C 21.907391 20.595391 22.415 21 23 21 C 23.585 21 24.092609 20.595391 24.224609 20.025391 L 26.308594 11 L 24.255859 11 L 23 16.439453 L 21.744141 11 L 19.691406 11 z M 8 13 L 9 13 C 9.552 13 10 13.448 10 14 L 10 18 C 10 18.552 9.552 19 9 19 L 8 19 L 8 13 z"></path>
                            </svg>
                        </Pressable> */}


                    </View>

                    {/* Title. */}
                    <Text style={ControllerStyle.textMedium}>EZ-RASSOR Controller</Text>

                    <View style={ControllerStyle.evenlySpaceIcons}>
                        {/* Stop-rover button. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => {
                                this.sendOperation(Robot.ALL, Operation.STOP);
                                this.setState({ ls: 0, rs: 0 });
                            }}
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="50%" width="50%">
								<path fill="#ffffff" d="M464 256A208 208 0 1 0 48 256a208 208 0 1 0 416 0zM0 256a256 256 0 1 1 512 0A256 256 0 1 1 0 256zm192-96l128 0c17.7 0 32 14.3 32 32l0 128c0 17.7-14.3 32-32 32l-128 0c-17.7 0-32-14.3-32-32l0-128c0-17.7 14.3-32 32-32z"/>
							</svg>
                        </Pressable>

                        {/*Paver Arm controls*/}
                        {/*<Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("Paver Arm Controller Screen", { currentIp: this.state.ip })}
                        >
                            <MaterialCommunityIcons
                                name="robot-industrial"
                                size={50}
                                color="#fff"
                            />
                        </Pressable>*/}

                        {/* Autonomy button — navigates to the full autonomy page. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("Autonomy Screen")}
                        >
                            <MaterialCommunityIcons
                                name="robot"
                                size={50}
                                color="#fff"
                            />
                        </Pressable>

                        {/* TODO make sure this passes the IP address to video screen  */}
                        {/* View-Video-Screen button.*/}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("Video Controller Screen", {
                                currentIp: this.state.ip,
								currentls: this.state.ls,
								currentrs: this.state.rs
                            })}
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 576 512" height="50%" width="50%">
								<path fill="#ffffff" d="M0 128C0 92.7 28.7 64 64 64l256 0c35.3 0 64 28.7 64 64l0 256c0 35.3-28.7 64-64 64L64 448c-35.3 0-64-28.7-64-64L0 128zM559.1 99.8c10.4 5.6 16.9 16.4 16.9 28.2l0 256c0 11.8-6.5 22.6-16.9 28.2s-23 5-32.9-1.6l-96-64L416 337.1l0-17.1 0-128 0-17.1 14.2-9.5 96-64c9.8-6.5 22.4-7.2 32.9-1.6z"/>
							</svg>
                        </Pressable>
                    </View>

                </View>

                {/* Body container. */}
                <View style={ControllerStyle.buttonLayoutContainer}>

                    {/* Wheel buttons. */}
                    <View style={ControllerStyle.wheelFunctionContainer}>


                        {/* Forwards button. */}
                        <Pressable style={ControllerStyle.upAndDownDPad}
                            onPressIn={async () => {  // Make the handler async to use await
                                this.startFlashing(); // Start flashing effect to represent loading
                                    
                                // Await the result of sendOperation to properly handle the async result
                                let success = await this.sendOperation(Robot.WHEELS, WheelOperation.FORWARD);
                                if (success == 1) {
                                    console.log("Command was Successfully sent");
                                    if (ls < 3)
                                        this.setState(prevState => ({ ls: prevState.ls + 1 }));
                                    if (rs < 3)
                                        this.setState(prevState => ({ rs: prevState.rs + 1 }));
                                    this.stopFlashing(); // Stop flashing effect
                                }
                                else if(success == 0) {
                                    console.log("Command could not be sent!");
                                    this.stopFlashing(); // Stop flashing effect
                                }
                                else if(success == -1)
                                    console.log("Interrupted Command! Cancelling prev command...");
                                else
                                    console.log("Unknown Error!");
                            }}
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="35%" width="35%">
								<path fill="#ffffff" d="M233.4 105.4c12.5-12.5 32.8-12.5 45.3 0l192 192c12.5 12.5 12.5 32.8 0 45.3s-32.8 12.5-45.3 0L256 173.3 86.6 342.6c-12.5 12.5-32.8 12.5-45.3 0s-12.5-32.8 0-45.3l192-192z"/>
							</svg>
                        </Pressable>


                        {/* Left, Stop, and Right buttons. */}
                        <View style={{ flex: 2, flexDirection: 'row' }}>

                            {/* Left button. */}
                            <Pressable style={ControllerStyle.dPadLeft}
                                onPressIn={async () => {
                                    this.startFlashing(); // Start flashing effect to represent loading
                                    
                                    // Await the result of sendOperation to properly handle the async result
                                    let success = await this.sendOperation(Robot.WHEELS, WheelOperation.LEFT);
                                    if (success == 1) {
                                        console.log("Command was Successfully sent");
                                        if (ls > -3)
                                            this.setState(prevState => ({ ls: prevState.ls - 1 }));
                                        if (rs < 3)
                                            this.setState(prevState => ({ rs: prevState.rs + 1 }));
                                        this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == 0) {
                                        console.log("Command could not be sent!");
                                        this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == -1)
                                        console.log("Interrupted Command! Cancelling prev command...");
                                    else
                                        console.log("Unknown Error!");
                                }}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 512" height="35%" width="35%">
									<path fill="#ffffff" d="M9.4 233.4c-12.5 12.5-12.5 32.8 0 45.3l192 192c12.5 12.5 32.8 12.5 45.3 0s12.5-32.8 0-45.3L77.3 256 246.6 86.6c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0l-192 192z"/>
								</svg>



                                <View style={ControllerStyle.circleOverlayVerticalLeft}>
                                    <View style={[
                                        ControllerStyle.circle,{
                                            backgroundColor: `rgb(${this.state.circleColorValue.r}, ${this.state.circleColorValue.g}, ${this.state.circleColorValue.b})`,
                                            shadowColor: "white", // Glow color
                                            shadowRadius: this.state.glowIntensity,
                                        },
                                        ls >= 3 ? ControllerStyle.positiveCircle : (ls <= -1 ? ControllerStyle.negativeCircle : {}) // top circle
                                    ]} />

                                    <View style={[
                                        ControllerStyle.circle,{
                                            backgroundColor: `rgb(${this.state.circleColorValue.r}, ${this.state.circleColorValue.g}, ${this.state.circleColorValue.b})`,
                                            shadowColor: "white", // Glow color
                                            shadowRadius: this.state.glowIntensity,
                                        },
                                        ls >= 2 ? ControllerStyle.positiveCircle : (ls <= -2 ? ControllerStyle.negativeCircle : {}) // middle circle
                                    ]} />

                                    <View style={[
                                        ControllerStyle.circle,{
                                            backgroundColor: `rgb(${this.state.circleColorValue.r}, ${this.state.circleColorValue.g}, ${this.state.circleColorValue.b})`,
                                            shadowColor: "white", // Glow color
                                            shadowRadius: this.state.glowIntensity,
                                        },
                                        ls >= 1 ? ControllerStyle.positiveCircle : (ls <= -3 ? ControllerStyle.negativeCircle : {}) // bottom circle
                                    ]} />
                                </View>

                            </Pressable>

                            {/* Stop button. */}
                            <Pressable style={[
                                    ControllerStyle.stopButton,
                                    { backgroundColor: `rgb(${this.state.colorValue.r}, ${this.state.colorValue.g}, ${this.state.colorValue.b})` }, // Use the interpolated RGB values
                                ]}
                                onPressIn={async () => {
                                    this.startFlashing(); // Start flashing effect to represent loading
                                    
                                    // Await the result of sendOperation to properly handle the async result
                                    let success = await this.sendOperation(Robot.WHEELS, WheelOperation.STOP);
                                    console.log("STOPPING ROVER...");
                                    if (success == 1) {
                                        console.log("Command was Successfully sent");
                                        this.stopFlashing(); // Stop flashing effect
                                        this.setState({ ls: 0, rs: 0, colorValue: { r: 128, g: 51, b: 51 } });
                                        this.fadeButton()
                                    }
                                    else if(success == 0) {
                                        console.log("Command could not be sent!");
                                        this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == -1)
                                        console.log("Interrupted Command! Cancelling prev command...");
                                    else
                                        console.log("Unknown Error!");
                                }}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 384 512" height="35%" width="35%">
									<path fill="#ffffff" d="M0 128C0 92.7 28.7 64 64 64H320c35.3 0 64 28.7 64 64V384c0 35.3-28.7 64-64 64H64c-35.3 0-64-28.7-64-64V128z"/>
								</svg>
                            </Pressable>

                            {/* Right button. */}
                            <Pressable style={ControllerStyle.dPadRight}
                                onPressIn={async () => {
                                    this.startFlashing(); // Start flashing effect to represent loading
                                
                                    // Await the result of sendOperation to properly handle the async result
                                    let success = await this.sendOperation(Robot.WHEELS, WheelOperation.RIGHT);
                                    if (success == 1) {
                                        console.log("Command was Successfully sent");
                                        
                                        if (ls < 3)
                                            this.setState(prevState => ({ ls: prevState.ls + 1 }));
                                        if (rs > -3)
                                            this.setState(prevState => ({ rs: prevState.rs - 1 }));
                                            this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == 0) {
                                        console.log("Command could not be sent!");
                                        this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == -1)
                                        console.log("Interrupted Command! Cancelling prev command...");
                                    else
                                        console.log("Unknown Error!");
                                }}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 512" height="35%" width="35%">
									<path fill="#ffffff" d="M310.6 233.4c12.5 12.5 12.5 32.8 0 45.3l-192 192c-12.5 12.5-32.8 12.5-45.3 0s-12.5-32.8 0-45.3L242.7 256 73.4 86.6c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0l192 192z"/>
								</svg>
                                <View style={ControllerStyle.circleOverlayVerticalRight}>
                                    <View style={[
                                        ControllerStyle.circle,{
                                            backgroundColor: `rgb(${this.state.circleColorValue.r}, ${this.state.circleColorValue.g}, ${this.state.circleColorValue.b})`,
                                            shadowColor: "white", // Glow color
                                            shadowRadius: this.state.glowIntensity,
                                        },
                                        rs >= 3 && ControllerStyle.positiveCircle, rs <= -1 && ControllerStyle.negativeCircle]} />
                                    <View style={[
                                        ControllerStyle.circle,{
                                            backgroundColor: `rgb(${this.state.circleColorValue.r}, ${this.state.circleColorValue.g}, ${this.state.circleColorValue.b})`,
                                            shadowColor: "white", // Glow color
                                            shadowRadius: this.state.glowIntensity,
                                        },
                                        rs >= 2 && ControllerStyle.positiveCircle, rs <= -2 && ControllerStyle.negativeCircle]} />
                                    <View style={[
                                        ControllerStyle.circle,{
                                            backgroundColor: `rgb(${this.state.circleColorValue.r}, ${this.state.circleColorValue.g}, ${this.state.circleColorValue.b})`,
                                            shadowColor: "white", // Glow color
                                            shadowRadius: this.state.glowIntensity,
                                        },
                                        rs >= 1 && ControllerStyle.positiveCircle, rs <= -3 && ControllerStyle.negativeCircle]} />
                                </View>
                            </Pressable>
                        </View>


                        

                        {/* Backwards button. */}
                        <Pressable style={ControllerStyle.upAndDownDPad}
                            onPressIn={ async () => {
                                    this.startFlashing(); // Start flashing effect to represent loading
                                
                                    // Await the result of sendOperation to properly handle the async result
                                    let success = await this.sendOperation(Robot.WHEELS, WheelOperation.BACKWARD);
                                    if (success == 1) {
                                        console.log("Command was Successfully sent");
                                        if (ls > -3)
                                            this.setState(prevState => ({ ls: prevState.ls - 1 }));
                                        if (rs > -3)
                                            this.setState(prevState => ({ rs: prevState.rs - 1 }));
                                        this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == 0) {
                                        console.log("Command could not be sent!");
                                        this.stopFlashing(); // Stop flashing effect
                                    }
                                    else if(success == -1)
                                        console.log("Interrupted Command! Cancelling prev command...");
                                    else
                                        console.log("Unknown Error!");
                            }}
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="35%" width="35%">
								<path fill="#ffffff" d="M233.4 406.6c12.5 12.5 32.8 12.5 45.3 0l192-192c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0L256 338.7 86.6 169.4c-12.5-12.5-32.8-12.5-45.3 0s-12.5 32.8 0 45.3l192 192z"/>
							</svg>
                        </Pressable>
                    </View>

                    {/* Drum/shoulder buttons. */}
                    <View style={ControllerStyle.drumFunctionContainer}>
                        <View style={{ flex: 8, width: '100%' }}>

                            {/* Top row of controls. */}
                            <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>
                                <View style={{ flexDirection: 'row', gap: '15px' }}>

                                    {/* Lower left-shoulder button. */}
                                    <View style={{width: 50, height: 50}}>
                                        {this.state.syncShoulders ? (<></>) : (

                                            <Pressable
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.FRONTARM, ShoulderOperation.LOWER);
                                                }}
                                                onPressOut={() => {
                                                    this.sendOperation(Robot.FRONTARM, ShoulderOperation.STOP);
                                                }}
                                            >
                                                <svg
                                                    xmlns="http://www.w3.org/2000/svg"
                                                    viewBox="0 0 512 512"
                                                    style={{ width: "100%", height: "100%" }}
                                                >
                                                    <path fill="#ffffff" path d="M256 0a256 256 0 1 0 0 512A256 256 0 1 0 256 0zM244.7 395.3l-112-112c-4.6-4.6-5.9-11.5-3.5-17.4s8.3-9.9 14.8-9.9l64 0 0-96c0-17.7 14.3-32 32-32l32 0c17.7 0 32 14.3 32 32l0 96 64 0c6.5 0 12.3 3.9 14.8 9.9s1.1 12.9-3.5 17.4l-112 112c-6.2 6.2-16.4 6.2-22.6 0z"/>
                                                </svg>
                                            </Pressable> ) }
                                    </View>

                                    {/* Raise left-shoulder button. */}
                                    <View style={{width: 50, height: 50}}>
                                    {this.state.syncShoulders ? (<></>) :  (
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.RAISE);
                                            }}
                                            onPressOut={() => {
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.STOP);
                                            }}
                                        >
											<svg
												xmlns="http://www.w3.org/2000/svg"
												viewBox="0 0 512 512"
												style={{ width: "100%", height: "100%" }}
											>
												<path fill="#ffffff" path d="M256 512A256 256 0 1 0 256 0a256 256 0 1 0 0 512zm11.3-395.3l112 112c4.6 4.6 5.9 11.5 3.5 17.4s-8.3 9.9-14.8 9.9l-64 0 0 96c0 17.7-14.3 32-32 32l-32 0c-17.7 0-32-14.3-32-32l0-96-64 0c-6.5 0-12.3-3.9-14.8-9.9s-1.1-12.9 3.5-17.4l112-112c6.2-6.2 16.4-6.2 22.6 0z"/>
											</svg>
                                        </Pressable>)
                                        }
                                    </View>

                                </View>

                                <View style = {{flexDirection: 'row', position: 'relative'}}>
                                {this.state.syncShoulders ? (
                                    <View style = {{width: 50, height: 50, marginHorizontal: 20}}>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.LOWER);
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.LOWER);
                                            }}

                                            onPressOut={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.STOP);
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.STOP);
                                            }}
                                        >
                                           <svg
                                                    xmlns="http://www.w3.org/2000/svg"
                                                    viewBox="0 0 512 512"
                                                    style={{ width: "100%", height: "100%" }}
                                                >
                                                    <path fill="#ffffff" path d="M256 0a256 256 0 1 0 0 512A256 256 0 1 0 256 0zM244.7 395.3l-112-112c-4.6-4.6-5.9-11.5-3.5-17.4s8.3-9.9 14.8-9.9l64 0 0-96c0-17.7 14.3-32 32-32l32 0c17.7 0 32 14.3 32 32l0 96 64 0c6.5 0 12.3 3.9 14.8 9.9s1.1 12.9-3.5 17.4l-112 112c-6.2 6.2-16.4 6.2-22.6 0z"/>
                                            </svg>
                                        </Pressable>
                                    </View>
                                ) : (<> </>)}

                                <Pressable
                                    style = {[{
                                        alignSelf: 'center',
                                        backgroundColor: this.state.syncShoulders ? 'green' : '#3a3d3d',
                                        borderRadius: 100,
                                        paddingVertical: 10,
                                        paddingHorizontal: 20,
                                        shadowColor:'black',
                                        shadowOffset: {width: 0, height: 0},
                                        shadowRadius: 10,
                                        shadowOpacity: .4
                                    }]}
                                    onPress={() => this.setState({syncShoulders : !this.state.syncShoulders})}
                                >

                                    <Text
                                        selectable = {false}
                                        style={{
                                            color:  'white',
                                            fontSize: '1.5vw',
                                            fontWeight: 'bold',
                                            }}
                                        >
                                            SYNC SHOULDERS
                                    </Text>

                                </Pressable>

                                {this.state.syncShoulders ? (
                                    <View style = {{width: 50, height: 50, marginHorizontal: 20}}>
                                        <Pressable
                                            onPressIn={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.RAISE);
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.RAISE);
                                            }}

                                            onPressOut={() => {
                                                this.sendOperation(Robot.BACKARM, ShoulderOperation.STOP);
                                                this.sendOperation(Robot.FRONTARM, ShoulderOperation.STOP);
                                            }}
                                        >
                                            <svg
                                                    xmlns="http://www.w3.org/2000/svg"
                                                    viewBox="0 0 512 512"
                                                    style={{ width: "100%", height: "100%" }}
                                                >
                                                    <path fill="#ffffff" path d="M256 512A256 256 0 1 0 256 0a256 256 0 1 0 0 512zm11.3-395.3l112 112c4.6 4.6 5.9 11.5 3.5 17.4s-8.3 9.9-14.8 9.9l-64 0 0 96c0 17.7-14.3 32-32 32l-32 0c-17.7 0-32-14.3-32-32l0-96-64 0c-6.5 0-12.3-3.9-14.8-9.9s-1.1-12.9 3.5-17.4l112-112c6.2-6.2 16.4-6.2 22.6 0z"/>
                                                </svg>
                                        </Pressable>
                                    </View>
                                ) : (<> </>)}
                                </View>




                                <View style={{ flexDirection: 'row', gap: '15px' }}>

                                    {/* Lower right-shoulder button. */}
                                    <View style={{width: 50, height: 50}}>
                                        {this.state.syncShoulders? (<></>) : (
                                            <Pressable
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.BACKARM, ShoulderOperation.LOWER);
                                                }}
                                                onPressOut={() => {
                                                    this.sendOperation(Robot.BACKARM, ShoulderOperation.STOP);
                                                }}
                                            >
                                                <svg
                                                    xmlns="http://www.w3.org/2000/svg"
                                                    viewBox="0 0 512 512"
                                                    style={{ width: "100%", height: "100%" }}
                                                >
                                                    <path fill="#ffffff" path d="M256 0a256 256 0 1 0 0 512A256 256 0 1 0 256 0zM244.7 395.3l-112-112c-4.6-4.6-5.9-11.5-3.5-17.4s8.3-9.9 14.8-9.9l64 0 0-96c0-17.7 14.3-32 32-32l32 0c17.7 0 32 14.3 32 32l0 96 64 0c6.5 0 12.3 3.9 14.8 9.9s1.1 12.9-3.5 17.4l-112 112c-6.2 6.2-16.4 6.2-22.6 0z"/>
                                                </svg>
                                            </Pressable>
                                        )}
                                    </View>

                                    {/* Raise right-shoulder button. */}
                                    <View style={{width: 50, height: 50}}>
                                        {this.state.syncShoulders ? (<></>) : (
                                            <Pressable
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.BACKARM, ShoulderOperation.RAISE);
                                                }}
                                                onPressOut={() => {
                                                    this.sendOperation(Robot.BACKARM, ShoulderOperation.STOP);
                                                }}
                                            >
                                                <svg
                                                    xmlns="http://www.w3.org/2000/svg"
                                                    viewBox="0 0 512 512"
                                                    style={{ width: "100%", height: "100%" }}
                                                >
                                                    <path fill="#ffffff" path d="M256 512A256 256 0 1 0 256 0a256 256 0 1 0 0 512zm11.3-395.3l112 112c4.6 4.6 5.9 11.5 3.5 17.4s-8.3 9.9-14.8 9.9l-64 0 0 96c0 17.7-14.3 32-32 32l-32 0c-17.7 0-32-14.3-32-32l0-96-64 0c-6.5 0-12.3-3.9-14.8-9.9s-1.1-12.9 3.5-17.4l112-112c6.2-6.2 16.4-6.2 22.6 0z"/>
                                                </svg>
                                            </Pressable>
                                        ) }
                                    </View>

                                </View>

                            </View>

                            {/* Thin image of rover. */}
                            <Image
                                style={ControllerStyle.image}
                                source={require('../../../assets/rassor.png')}
                                resizeMode="contain"
                            />

                            {/* Bottom row of controls. */}
                            <View style={{ flexDirection: "row", justifyContent: 'space-between' }}>

                                <View style={{ flexDirection: "row", gap: "15px" }}>

                                    {/* Dump left-bucket button. */}
                                    <View style={{width: 50, height: 50}}>
                                        {this.state.syncDrums? (<></>) : (
                                            <Pressable
                                            style = {[{ fill: this.state.toggleLeftDump ? 'green' : 'white', }]}
                                            onPressIn={() => {
                                                this.setState(prevState => ({
                                                    toggleLeftDump: !prevState.toggleLeftDump,
                                                    toggleLeftDig: false,
                                                }), () => {
                                                    this.state.toggleLeftDump
                                                        ? this.sendOperation(Robot.FRONTDRUM, DrumOperation.DUMP)
                                                        : this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                                });
                                            }}
                                        >
											<svg xmlns="http://www.w3.org/2000/svg"
											    viewBox="0 0 512 512"
											    style={{
                                                    width: '100%', height: '100%', display: 'inline'
                                                }}
											>
												<path path d="M48.5 224L40 224c-13.3 0-24-10.7-24-24L16 72c0-9.7 5.8-18.5 14.8-22.2s19.3-1.7 26.2 5.2L98.6 96.6c87.6-86.5 228.7-86.2 315.8 1c87.5 87.5 87.5 229.3 0 316.8s-229.3 87.5-316.8 0c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0c62.5 62.5 163.8 62.5 226.3 0s62.5-163.8 0-226.3c-62.2-62.2-162.7-62.5-225.3-1L185 183c6.9 6.9 8.9 17.2 5.2 26.2s-12.5 14.8-22.2 14.8L48.5 224z"/>
											</svg>
                                        </Pressable>
                                        )}
                                    </View>

                                    {/* Dig left-bucket button. */}
                                    <View style={{width: 50, height: 50}}>

                                    {this.state.syncDrums? (<></>) : (

                                        <Pressable
                                            style = {[{ fill: this.state.toggleLeftDig ? 'green' : 'white', }]}
                                            onPressIn={() => {
                                                this.setState(prevState => ({
                                                    toggleLeftDig: !prevState.toggleLeftDig,
                                                    toggleLeftDump: false,
                                                }), () => {
                                                    this.state.toggleLeftDig
                                                        ? this.sendOperation(Robot.FRONTDRUM, DrumOperation.DIG)
                                                        : this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                                });
                                            }}
                                        >

                                            <svg xmlns="http://www.w3.org/2000/svg"
												viewBox="0 0 512 512"
												style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path  path d="M463.5 224l8.5 0c13.3 0 24-10.7 24-24l0-128c0-9.7-5.8-18.5-14.8-22.2s-19.3-1.7-26.2 5.2L413.4 96.6c-87.6-86.5-228.7-86.2-315.8 1c-87.5 87.5-87.5 229.3 0 316.8s229.3 87.5 316.8 0c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0c-62.5 62.5-163.8 62.5-226.3 0s-62.5-163.8 0-226.3c62.2-62.2 162.7-62.5 225.3-1L327 183c-6.9 6.9-8.9 17.2-5.2 26.2s12.5 14.8 22.2 14.8l119.5 0z"/>
											</svg>
                                        </Pressable>
                                    )}
                                    </View>

                                </View>

                                <View style = {{flexDirection: 'row', position: 'relative'}}>

                                    {this.state.syncDrums ? (
                                        <View style = {{width: 50, height: 50, marginHorizontal: 20}}>
                                            <Pressable
                                            style = {[{ fill: this.state.toggleSyncDig ? 'green' : 'white', }]}

                                            onPressIn={() => {
                                            this.setState(prevState => ({
                                                toggleSyncDig: !prevState.toggleSyncDig,
                                                toggleSyncDump: false,
                                            }),

                                            () => {
                                                this.state.toggleSyncDig
                                                    ? this.sendOperation(Robot.FRONTDRUM, DrumOperation.DIG)
                                                    : this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                            });
                                            }}
                                            >
                                            <svg xmlns="http://www.w3.org/2000/svg"
											    viewBox="0 0 512 512"
											    style={{ width: '100%', height: '100%', display: 'inline',}}
											>
												<path path d="M48.5 224L40 224c-13.3 0-24-10.7-24-24L16 72c0-9.7 5.8-18.5 14.8-22.2s19.3-1.7 26.2 5.2L98.6 96.6c87.6-86.5 228.7-86.2 315.8 1c87.5 87.5 87.5 229.3 0 316.8s-229.3 87.5-316.8 0c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0c62.5 62.5 163.8 62.5 226.3 0s62.5-163.8 0-226.3c-62.2-62.2-162.7-62.5-225.3-1L185 183c6.9 6.9 8.9 17.2 5.2 26.2s-12.5 14.8-22.2 14.8L48.5 224z"/>
											</svg>
                                            </Pressable>
                                        </View>
                                    ) : (<> </>)}


                                    <Pressable
                                        style={[{
                                            alignSelf: 'center',
                                            backgroundColor: this.state.syncDrums ? 'green' : '#3a3d3d',
                                            borderRadius: 100,
                                            paddingVertical: 10,
                                            paddingHorizontal: 20,
                                            shadowColor:'black',
                                            shadowOffset: {width: 0, height: 0},
                                            shadowRadius: 10,
                                            shadowOpacity: .4
                                        }]}
                                        onPress={() => this.setState({
                                            syncDrums : !this.state.syncDrums,
                                            toggleLeftDig: false,
                                            toggleLeftDump: false,
                                            toggleRightDig: false,
                                            toggleRightDump: false,
                                            toggleSyncDig: false,
                                            toggleSyncDump: false
                                        },
                                        () => this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP) )}
                                    >
                                        <Text
                                            selectable = {false}
                                            style={{
                                                color:  'white',
                                                fontSize: '1.5vw',
                                                fontWeight: 'bold'
                                            }}
                                        >
                                                SYNC DRUMS
                                        </Text>
                                    </Pressable>


                                    {this.state.syncDrums ? (
                                    <View style = {{width: 50, height: 50, marginHorizontal: 20}}>
                                        <Pressable
                                            style = {[{ fill: this.state.toggleSyncDump ? 'green' : 'white', }]}
                                            onPressIn={() => {
                                            this.setState(prevState => ({
                                                toggleSyncDump: !prevState.toggleSyncDump,
                                                toggleSyncDig: false,
                                            }), () => {
                                                this.state.toggleSyncDump
                                                    ? this.sendOperation(Robot.FRONTDRUM, DrumOperation.DUMP)
                                                    : this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                            });
                                        }}
                                        >
                                            <svg xmlns="http://www.w3.org/2000/svg"
												viewBox="0 0 512 512"
												style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path path d="M463.5 224l8.5 0c13.3 0 24-10.7 24-24l0-128c0-9.7-5.8-18.5-14.8-22.2s-19.3-1.7-26.2 5.2L413.4 96.6c-87.6-86.5-228.7-86.2-315.8 1c-87.5 87.5-87.5 229.3 0 316.8s229.3 87.5 316.8 0c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0c-62.5 62.5-163.8 62.5-226.3 0s-62.5-163.8 0-226.3c62.2-62.2 162.7-62.5 225.3-1L327 183c-6.9 6.9-8.9 17.2-5.2 26.2s12.5 14.8 22.2 14.8l119.5 0z"/>
											</svg>
                                        </Pressable>
                                    </View>
                                ) : (<> </>)}
                                </View>

                                <View style={{ flexDirection: 'row', gap: 15, alignItems: 'flex-end' }}>

                                    {/* Dump right-bucket button. */}
                                    <View style={{width: 50, height: 50}}>

                                    {this.state.syncDrums? (<></>) : (
                                        <Pressable
                                        style = {[{ fill: this.state.toggleRightDump ? 'green' : 'white', }]}
                                        onPressIn={() => {
                                            this.setState(prevState => ({
                                                toggleRightDump: !prevState.toggleRightDump,
                                                toggleRightDig: false,
                                            }), () => {
                                                this.state.toggleRightDump
                                                    ? this.sendOperation(Robot.FRONTDRUM, DrumOperation.DUMP)
                                                    : this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                            });
                                        }}
                                        >
											<svg xmlns="http://www.w3.org/2000/svg"
											viewBox="0 0 512 512"
											style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path path d="M48.5 224L40 224c-13.3 0-24-10.7-24-24L16 72c0-9.7 5.8-18.5 14.8-22.2s19.3-1.7 26.2 5.2L98.6 96.6c87.6-86.5 228.7-86.2 315.8 1c87.5 87.5 87.5 229.3 0 316.8s-229.3 87.5-316.8 0c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0c62.5 62.5 163.8 62.5 226.3 0s62.5-163.8 0-226.3c-62.2-62.2-162.7-62.5-225.3-1L185 183c6.9 6.9 8.9 17.2 5.2 26.2s-12.5 14.8-22.2 14.8L48.5 224z"/>
											</svg>
                                        </Pressable>
                                    )}

                                    </View>

                                    {/* Dig right-bucket button. */}
                                    <View style ={{ width: 50, height: 50}}>
                                    {this.state.syncDrums? (<></>) : (
                                        <Pressable
                                        style = {[{ fill: this.state.toggleRightDig ? 'green' : 'white', }]}
                                        onPressIn={() => {
                                            this.setState(prevState => ({
                                                toggleRightDig: !prevState.toggleRightDig,
                                                toggleRightDump: false
                                            }), () => {
                                                this.state.toggleRightDig
                                                    ? this.sendOperation(Robot.FRONTDRUM, DrumOperation.DIG)
                                                    : this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                            });
                                        }}
                                        >
											<svg xmlns="http://www.w3.org/2000/svg"
											viewBox="0 0 512 512"
											style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path   path d="M463.5 224l8.5 0c13.3 0 24-10.7 24-24l0-128c0-9.7-5.8-18.5-14.8-22.2s-19.3-1.7-26.2 5.2L413.4 96.6c-87.6-86.5-228.7-86.2-315.8 1c-87.5 87.5-87.5 229.3 0 316.8s229.3 87.5 316.8 0c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0c-62.5 62.5-163.8 62.5-226.3 0s-62.5-163.8 0-226.3c62.2-62.2 162.7-62.5 225.3-1L327 183c-6.9 6.9-8.9 17.2-5.2 26.2s12.5 14.8 22.2 14.8l119.5 0z"/>
											</svg>
                                        </Pressable>
                                    )}
                                    </View>

                                </View>

                            </View>

                        </View>
                    </View>

                </View>

            </View>
        );
    }
}
