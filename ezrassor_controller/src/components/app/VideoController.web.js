import { Pressable, StyleSheet, Text, View, Alert, TouchableHighlight } from 'react-native';
import * as React from 'react';
import Modal from 'react-native-modal';
import EZRASSOR from '../../../src/api/ezrassor-service';
import EZARM from '../../../src/api/ezrassor-service-paver-arm'
import { Robot, ShoulderOperation, DrumOperation, WheelOperation, Operation } from '../../../src/enumerations/robot-commands';
import { isIpReachable } from '../../functionality/connection';
import ControllerStyle from '../../../src/styles/controller';
import VideoStyle from '../../../src/styles/video';
import { useRoute } from '@react-navigation/native';

const VIDEO_EXT_STANDARD = '/video_feed';
const DETECTION_POLLING_EXTENSION = '/is_detection';
const DETECTION_POLLING_INTERVAL = 5000;

const CONNECTION_POLLING_INTERVAL = 2000;
const CONNECTION_POLLING_TIMEOUT = 4000;
const CONNECTION_POLLING_VERBOSE = false;

export default class VideoController extends React.Component {

    /**
     * Constructor for VideoController componenet
     * ip variable is part of props passed to this component from ControllerScreen component 
     */
    constructor(props) {
        super(props);

        this.pollForDetection = this.pollForDetection.bind(this);

        this.state = {
            autonomyModalVisible: false,
            infoModalVisible: false,
            devModalVisible: false,
            ls:  this.props.route.params.currentls,
            rs:  this.props.route.params.currentrs,
            ipModal: false,
            xyModal: false,
            isLoading: true,
            isDetecting: false,
            control: 0,
            circleColorValue: { r: 91, g: 91, b: 91 }, // Light color
            xy: '0,0',
            ip: '',
            videoIp: '',
            pollingRoute: '',
        };
        this.circleInterval = null;

        this.EZARM = new EZARM(this.state.ip);
        this.EZRASSOR = new EZRASSOR(this.state.ip);

    }

    set detectionReport(report) {

    }

    /**
     * Load assets required for this component and check connection to IP address
     */
    async componentDidMount() {
		await this.getIpFromStorage();
        const currentIp = this.props.route.params.currentIp;
        // important line: sets the IP address for current EZRASSOR instance
        this.EZRASSOR.host = currentIp;

        const base = currentIp.startsWith('http') ? currentIp : `http://${currentIp}`;

        this.setState({
            ip:           currentIp,
            videoIp:      `${base}${VIDEO_EXT_STANDARD}`,
            pollingRoute: `${base}${DETECTION_POLLING_EXTENSION}`,
        });

        // this.state.videoIp = this.props.route.params.currentIp + this.videoExtension;
        console.log('Set videoIp to: ' + this.state.videoIp);
        console.log('Set polling route to: ' + this.state.pollingRoute);

        this.setState({ isLoading: false });

        this._unsubscribe = this.props.navigation.addListener('focus', async () => {
            this.getIpFromStorage();
        });

        // Set up the connection polling logic.
        let pollCount = 0;
        /*this.connectionPoller = setInterval(async () => {
            if (await isIpReachable(this.state.ip, CONNECTION_POLLING_TIMEOUT)) {
                if (CONNECTION_POLLING_VERBOSE) {
                    console.log(`Connection poll attempt ${++pollCount} succeeded...`);
                }
            } else {
                console.log(`Dropped connection from ${this.state.ip}... redirecting to connection screen.`);
                this.props.navigation.replace('Connection Status Screen', { screen: 'roverDisconnected' });
            }
        }, CONNECTION_POLLING_INTERVAL);

        const pollInterval = (CONNECTION_POLLING_INTERVAL / 1000.0).toFixed(2);
        const pollTimeout = (CONNECTION_POLLING_TIMEOUT / 1000.0).toFixed(2);
        console.log(`Polling every ${pollInterval}s with a ${pollTimeout}s timeout.`);*/

    }

    setAutonomyModalVisible(invisible) {
        this.setState({ autonomyModalVisible: invisible });
    }

    setInfoModalVisible(visible) {
        this.setState({ infoModalVisible: visible });
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
     * Retrieves IP addreess saved previously in AsyncStorage 
     */
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

    /**
     * Set `this.state.ip` and `this.EZRASSOR.host` to the specified IP + port.
     * 
     * @param {string} ip IP + port
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


    /**
     * Update animation frame before processing click so that opacity can change on click.
     */
    // TODO: verify this sends coordiantes of paver to this function and handle it on the arm service
    sendPickUpOperation(part, operation) {
        requestAnimationFrame(() => {
            this.EZARM.executeRobotCommand(part, operation)
        });
    }

    /**
     * Stop and start the paver detection polling service
     * Call function to change video extension, 
     * causing the component to re-render with the new soucre for video feed
     */
    togglePaverDetection() {
        if (this.state.isDetecting) {
            this.state.isDetecting = false;
            this.stopDetectionPolling();
        }
        else {
            this.state.isDetecting = true;
            this.startDetectionPolling();
        }
    }

    /**
      * Poll Controller Server Detection API to confirm a paver is still in view to pick
      */
    async confirmDetection(x, y, z) {
        const response = await fetch(this.state.pollingRoute)
            .then((response) => response.json())
            .then((json) => console.log(json));

        pickUpOperation = json.x.toString() + json.y.toString() + json.z.toString();

        if (json.hasOwnProperty('x')) {
            // TODO: check that this is the same pave the user wanted to pick up
            // Figure out how to verify that it is the same paver the user said to pick up
            //this.sendPickUpOperation(Arm.AUTONOMY, pickUpOperation);
            console.log("paver to pick up confirmed")
        }
        else {
            console.log("paver to pick up no longer in view")
        }
    }

    /**
     * Handles request to pick up paver
     * 
     */
    handlePickUpRequest(x, y, z) {
        // TODO: add this to send command to arm for picking up paver
        // Need to finish aruco postion calculation first
        // this.EZRASSOR.sendOperation(Robot.WHEELS, WheelOperation.STOP);
        // this.confirmDetection(x, y, z);
    }

    /**
     * Creates and shows dialog box with paver detection info
     */
    createPaverNotification(x, y, z) {
        Alert.alert('Paver Detected at ' + x + ', ' + y + ', ' + z, 'choose arm action', [
            {
                text: 'Ignore',
                onPress: () => this.startDetectionPolling(), // Restart detection polling if user chooses to ingore current paver
                style: 'cancel',
            },
            { text: 'Pick Up (Not implimented yet)', onPress: () => this.handlePickUpRequest(x, y, z) }, // Prepare to pick up paver
        ]);
    }

    /**
     * Starts polling to Controller Server API for detection of Pavers.
     * Sets interval polling function will be called at.
     */
    startDetectionPolling() {
        console.log("Starting Detection Polling");

        this.detectionTask = setInterval(this.pollForDetection, DETECTION_POLLING_INTERVAL);
    }

    /**
     * Stops polling to Controller Server API for detection of Pavers.
     */
    stopDetectionPolling() {
        console.log("Stopping Detection Polling");
        clearInterval(this.detectionTask);
    }

    /**
      * Poll Controller Server API to check for detection of Pavers.
      * Makes API requests at specified interval
      */
    async pollForDetection() {
        console.log("Polling for detection at: " + this.state.pollingRoute);

        let response = await fetch(this.state.pollingRoute);
        let data = await response.json();

        console.log(data);

        // JSON response contains detection report, stop detection polling and create notification
        if (data.hasOwnProperty('x')) {
            this.stopDetectionPolling();
            this.createPaverNotification(data.x, data.y, data.z);
        }
        else {
            console.log("Detection report did not have a coordinate");
        }
    }

    // Stop detection polling and navigate to standard controller screen
    switchToControllerScreen() {
        if (this.state.isDetecting) {
            this.stopDetectionPolling();
            this.state.isDetecting = false;
        }
		this.props.navigation.replace("Controller Screen", {
			currentls: this.state.ls,
			currentrs: this.state.rs
        });
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
        const { rs, ls } = this.state

        // Use WebView to render the webpage hosting the live video streaming from EZRASSOR
        return (
            <View style={styles.container}>

                {/* Video Controller page Info popup modal. */}
                <Modal
                    supportedOrientations={['landscape']}
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.infoModalVisible}
                    onSwipeComplete={() => this.setInfoModalVisible(false)}
                    swipeDirection={["down", "up", "left", "right"]}
                    onRequestClose={() => this.setInfoModalVisible(false)}
                >

                    <TouchableHighlight style={{ justifyContent: 'space-evenly', alignItems: 'center'}}>
                        <View>
                            <View style={{ flexDirection: 'row', justifyContent: 'center', marginBottom: 25, paddingBottom:150 }}>
                                <Text style={ControllerStyle.textLarge}>Controller Help</Text>
                            </View>

                            <View style={{ flexDirection: 'row', justifyContent: 'center', alignItems: 'center', marginHorizontal: "0.5%"}}>
                                <View style={{justifyContent: 'space-between', alignItems: 'center', paddingHorizontal:'1%'}} >
                                    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" height="18%" width="18%">
                                        <path fill="#ffffff" d="M54.2 202.9C123.2 136.7 216.8 96 320 96s196.8 40.7 265.8 106.9c12.8 12.2 33 11.8 45.2-.9s11.8-33-.9-45.2C549.7 79.5 440.4 32 320 32S90.3 79.5 9.8 156.7C-2.9 169-3.3 189.2 8.9 202s32.5 13.2 45.2 .9zM320 256c56.8 0 108.6 21.1 148.2 56c13.3 11.7 33.5 10.4 45.2-2.8s10.4-33.5-2.8-45.2C459.8 219.2 393 192 320 192s-139.8 27.2-190.5 72c-13.3 11.7-14.5 31.9-2.8 45.2s31.9 14.5 45.2 2.8c39.5-34.9 91.3-56 148.2-56zm64 160a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z"/>
                                    </svg>
                                    <View style={{paddingTop: '10%'}}>
                                        <Text style={ControllerStyle.textSmall}>
                                            Connects to the server with input string: {'\n'}
                                            IP : PORT
                                        </Text>
                                    </View>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{justifyContent: 'space-evenly', alignItems: 'center', paddingHorizontal:'1%'}} >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" path fill='#ffff' height="18%" width="18%">
                                    <path d="M17,22V20H20V17H22V20.5C22,20.89 21.84,21.24 21.54,21.54C21.24,21.84 20.89,22 20.5,22H17M7,22H3.5C3.11,22 2.76,21.84 2.46,21.54C2.16,21.24 2,20.89 2,20.5V17H4V20H7V22M17,2H20.5C20.89,2 21.24,2.16 21.54,2.46C21.84,2.76 22,3.11 22,3.5V7H20V4H17V2M7,2V4H4V7H2V3.5C2,3.11 2.16,2.76 2.46,2.46C2.76,2.16 3.11,2 3.5,2H7M13,17.25L17,14.95V10.36L13,12.66V17.25M12,10.92L16,8.63L12,6.28L8,8.63L12,10.92M7,14.95L11,17.25V12.66L7,10.36V14.95M18.23,7.59C18.73,7.91 19,8.34 19,8.91V15.23C19,15.8 18.73,16.23 18.23,16.55L12.75,19.73C12.25,20.05 11.75,20.05 11.25,19.73L5.77,16.55C5.27,16.23 5,15.8 5,15.23V8.91C5,8.34 5.27,7.91 5.77,7.59L11.25,4.41C11.5,4.28 11.75,4.22 12,4.22C12.25,4.22 12.5,4.28 12.75,4.41L18.23,7.59Z" />
                                </svg>
                                    <View style={{paddingTop: '10%'}}>
                                        <Text style={ControllerStyle.textSmall}>
                                            Start/Stop Paver Detection notification
                                        </Text>
                                    </View>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{justifyContent: 'center', margin: 'auto', alignItems: 'center', paddingHorizontal:'1%'}} >
                                    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="18%" width="18%" >
                                        <path fill="#ffffff" d="M464 256A208 208 0 1 0 48 256a208 208 0 1 0 416 0zM0 256a256 256 0 1 1 512 0A256 256 0 1 1 0 256zm192-96l128 0c17.7 0 32 14.3 32 32l0 128c0 17.7-14.3 32-32 32l-128 0c-17.7 0-32-14.3-32-32l0-128c0-17.7 14.3-32 32-32z"/>\
                                    </svg>
                                    <View style={{paddingTop: '10%'}}>
                                        <Text style={ControllerStyle.textSmall}>
                                            Emergency stop for manual movements
                                        </Text>
                                    </View>
                                </View>

                                {/* Vertical separating bar.
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <MaterialCommunityIcons style={{ textAlign: 'center' }} name="robot-industrial" size={32} color="#fff" />
                                    <Text style={ControllerStyle.textSmall}>
                                        Navigate to Paver Arm Controller
                                    </Text>
                                </View>*/}

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%", alignItems: 'center', paddingBottom: '15%' }}></View>

                                <View style={{justifyContent: 'center', alignItems: 'center', paddingHorizontal:'1%'}} >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" path fill="#ffff" height="20%" width="20%">
                                    <path d="M38.8 5.1C28.4-3.1 13.3-1.2 5.1 9.2S-1.2 34.7 9.2 42.9l592 464c10.4 8.2 25.5 6.3 33.7-4.1s6.3-25.5-4.1-33.7l-86.4-67.7 13.8 9.2c9.8 6.5 22.4 7.2 32.9 1.6s16.9-16.4 16.9-28.2l0-256c0-11.8-6.5-22.6-16.9-28.2s-23-5-32.9 1.6l-96 64L448 174.9l0 17.1 0 128 0 5.8-32-25.1L416 128c0-35.3-28.7-64-64-64L113.9 64 38.8 5.1zM407 416.7L32.3 121.5c-.2 2.1-.3 4.3-.3 6.5l0 256c0 35.3 28.7 64 64 64l256 0c23.4 0 43.9-12.6 55-31.3z"/>
                                </svg>
                                    <View style={{paddingTop: '10%'}}>
                                        <Text style={ControllerStyle.textSmall}>
                                            Stop Live Stream
                                        </Text>
                                    </View>
                                </View>
                            </View>
                        </View>
                    </TouchableHighlight>
				{/* Modal X button. */}
				<Pressable
					style={[ControllerStyle.buttonModalContainer]}
					onPress={() => {
						this.setInfoModalVisible(false);
					}}
            	>
					<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="36" height="36">
						<line x1="18" y1="6" x2="6" y2="18" stroke="black" stroke-width="2" stroke-linecap="round"/>
						<line x1="6" y1="6" x2="18" y2="18" stroke="black" stroke-width="2" stroke-linecap="round"/>
					</svg>
				</Pressable>
                </Modal>

                <View style={styles.webViewContainer}>

                    {this.state.imageFailed ? (
                        <View style={{marginTop: '10%',  justifyContent: 'center', alignItems: 'center'}}>
                            <View style={[{backgroundColor: '#2e3030', borderRadius: 15, paddingHorizontal: 20, paddingVertical: 10}]}>
                                <Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.titleText, { fontSize: '5.0vw', justifyContent: 'center', alignItems: 'center' }]}>
                                    Video Feed Failed To Load
                                </Text>
                            </View>
                        </View>
                    ) : (
                        <img
                        src={this.state.videoIp}
                        style={styles.webView}
                        onError={() => this.setState({ imageFailed: true })}
                        />
                    )}

                    {/*Nav Bar*/}
                    <View style={styles.overlayedMenu}>
                        {/* Controller screen top row controls. */}
                        <View style={VideoStyle.headerContainer}>
                            {/* Info button. */}
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => { this.setInfoModalVisible(true); }}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="100%" width="100%">
								    <path fill="#ffffff" d="M256 512A256 256 0 1 0 256 0a256 256 0 1 0 0 512zM216 336l24 0 0-64-24 0c-13.3 0-24-10.7-24-24s10.7-24 24-24l48 0c13.3 0 24 10.7 24 24l0 88 8 0c13.3 0 24 10.7 24 24s-10.7 24-24 24l-80 0c-13.3 0-24-10.7-24-24s10.7-24 24-24zm40-208a32 32 0 1 1 0 64 32 32 0 1 1 0-64z"/>
							    </svg>
                            </Pressable>

                            {/* Set-IP button. */}
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => this.props.navigation.replace("IPConnect Screen")}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" height="100%" width="100%">
								    <path fill="#ffffff" d="M54.2 202.9C123.2 136.7 216.8 96 320 96s196.8 40.7 265.8 106.9c12.8 12.2 33 11.8 45.2-.9s11.8-33-.9-45.2C549.7 79.5 440.4 32 320 32S90.3 79.5 9.8 156.7C-2.9 169-3.3 189.2 8.9 202s32.5 13.2 45.2 .9zM320 256c56.8 0 108.6 21.1 148.2 56c13.3 11.7 33.5 10.4 45.2-2.8s10.4-33.5-2.8-45.2C459.8 219.2 393 192 320 192s-139.8 27.2-190.5 72c-13.3 11.7-14.5 31.9-2.8 45.2s31.9 14.5 45.2 2.8c39.5-34.9 91.3-56 148.2-56zm64 160a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z"/>
							    </svg>
                            </Pressable>


                            {/* Title. */}
                            <Text style={ControllerStyle.textMedium}>EZ-RASSOR Controller</Text>


                            {/*Paver Arm controls
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => this.props.navigation.replace("Paver Arm Controller Screen", { currentIp: this.state.ip })}
                            >
                                <MaterialCommunityIcons
                                    name="robot-industrial"
                                    size={50}
                                    color="#fff"
                                />
                            </Pressable>*/}


                            {/* Toggle paver detection button. */}
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => this.togglePaverDetection()}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" height="100%" width="100%" path fill='#ffff'>
                                    <path d="M17,22V20H20V17H22V20.5C22,20.89 21.84,21.24 21.54,21.54C21.24,21.84 20.89,22 20.5,22H17M7,22H3.5C3.11,22 2.76,21.84 2.46,21.54C2.16,21.24 2,20.89 2,20.5V17H4V20H7V22M17,2H20.5C20.89,2 21.24,2.16 21.54,2.46C21.84,2.76 22,3.11 22,3.5V7H20V4H17V2M7,2V4H4V7H2V3.5C2,3.11 2.16,2.76 2.46,2.46C2.76,2.16 3.11,2 3.5,2H7M13,17.25L17,14.95V10.36L13,12.66V17.25M12,10.92L16,8.63L12,6.28L8,8.63L12,10.92M7,14.95L11,17.25V12.66L7,10.36V14.95M18.23,7.59C18.73,7.91 19,8.34 19,8.91V15.23C19,15.8 18.73,16.23 18.23,16.55L12.75,19.73C12.25,20.05 11.75,20.05 11.25,19.73L5.77,16.55C5.27,16.23 5,15.8 5,15.23V8.91C5,8.34 5.27,7.91 5.77,7.59L11.25,4.41C11.5,4.28 11.75,4.22 12,4.22C12.25,4.22 12.5,4.28 12.75,4.41L18.23,7.59Z" />
                                </svg>
                            </Pressable>


                            {/* Back to Controller Screen button.*/}
                            <Pressable
                                style={ControllerStyle.icons}
                                onPress={() => this.switchToControllerScreen()}
                            >
                                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" height="100%" width="100%">
                                    <path fill="#ffffff" d="M38.8 5.1C28.4-3.1 13.3-1.2 5.1 9.2S-1.2 34.7 9.2 42.9l592 464c10.4 8.2 25.5 6.3 33.7-4.1s6.3-25.5-4.1-33.7l-86.4-67.7 13.8 9.2c9.8 6.5 22.4 7.2 32.9 1.6s16.9-16.4 16.9-28.2l0-256c0-11.8-6.5-22.6-16.9-28.2s-23-5-32.9 1.6l-96 64L448 174.9l0 17.1 0 128 0 5.8-32-25.1L416 128c0-35.3-28.7-64-64-64L113.9 64 38.8 5.1zM407 416.7L32.3 121.5c-.2 2.1-.3 4.3-.3 6.5l0 256c0 35.3 28.7 64 64 64l256 0c23.4 0 43.9-12.6 55-31.3z"/>
                                </svg>
                            </Pressable>
                        </View>
                    </View>

                    {/* Wheel, Shoulder, and Drum buttons */}
                    <View style={styles.buttonContainer}>

                        {/* Forwards button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
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
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="70%" width="70%">
								<path fill="#ffffff" d="M233.4 105.4c12.5-12.5 32.8-12.5 45.3 0l192 192c12.5 12.5 12.5 32.8 0 45.3s-32.8 12.5-45.3 0L256 173.3 86.6 342.6c-12.5 12.5-32.8 12.5-45.3 0s-12.5-32.8 0-45.3l192-192z"/>
							</svg>
                        </Pressable>

                        {/* Backwards button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
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
                            resizeMode="contain"
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="70%" width="70%">
								<path fill="#ffffff" d="M233.4 406.6c12.5 12.5 32.8 12.5 45.3 0l192-192c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0L256 338.7 86.6 169.4c-12.5-12.5-32.8-12.5-45.3 0s-12.5 32.8 0 45.3l192 192z"/>
							</svg>
                        </Pressable>

                        {/* Drum/shoulder buttons. */}
                        <View style={VideoStyle.drumFunctionContainer}>
                            <View style={{ flex: 4 }}>

                                {/* Top row of controls. */}
                                <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>
                                    <View style={{ flexDirection: 'column' }}>
                                        <View style={{ flexDirection: 'row', gap: '15px' }}>

                                            {/* Raise left-shoulder button. */}
                                            <View style={{width: 30, height: 30, flex: 1}}>
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
                                                </Pressable>
                                            </View>

                                            {/* Lower left-shoulder button. */}
                                            <View style={{width: 30, height: 30, flex: 1}}>
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
                                                </Pressable>
                                            </View>

                                        </View>

                                        <View style={{ flexDirection: "row", gap: '15px' }}>

                                            {/* Dump left-bucket button. */}
                                            <View style={{width: 30, height: 30, flex: 1}}>
                                                <Pressable
                                                    onPressIn={() => {
                                                        this.sendOperation(
                                                            Robot.FRONTDRUM,
                                                            DrumOperation.DUMP
                                                        );
                                                    }}
                                                    onPressOut={() => {
                                                        this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                                    }}
                                                >
											<svg xmlns="http://www.w3.org/2000/svg"
												viewBox="0 0 512 512"
												style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path fill="#ffffff" path d="M48.5 224L40 224c-13.3 0-24-10.7-24-24L16 72c0-9.7 5.8-18.5 14.8-22.2s19.3-1.7 26.2 5.2L98.6 96.6c87.6-86.5 228.7-86.2 315.8 1c87.5 87.5 87.5 229.3 0 316.8s-229.3 87.5-316.8 0c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0c62.5 62.5 163.8 62.5 226.3 0s62.5-163.8 0-226.3c-62.2-62.2-162.7-62.5-225.3-1L185 183c6.9 6.9 8.9 17.2 5.2 26.2s-12.5 14.8-22.2 14.8L48.5 224z"/>
											</svg>
                                                </Pressable>
                                            </View>

                                            {/* Dig left-bucket button. */}
                                            <View style={{width: 30, height: 30, flex: 1}}>
                                                <Pressable
                                                    onPressIn={() => {
                                                        this.sendOperation(
                                                            Robot.FRONTDRUM,
                                                            DrumOperation.DIG
                                                        );
                                                    }}
                                                    onPressOut={() => {
                                                        this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                                    }}
                                                >
											<svg xmlns="http://www.w3.org/2000/svg"
												viewBox="0 0 512 512"
												style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path fill="#ffffff"  path d="M463.5 224l8.5 0c13.3 0 24-10.7 24-24l0-128c0-9.7-5.8-18.5-14.8-22.2s-19.3-1.7-26.2 5.2L413.4 96.6c-87.6-86.5-228.7-86.2-315.8 1c-87.5 87.5-87.5 229.3 0 316.8s229.3 87.5 316.8 0c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0c-62.5 62.5-163.8 62.5-226.3 0s-62.5-163.8 0-226.3c62.2-62.2 162.7-62.5 225.3-1L327 183c-6.9 6.9-8.9 17.2-5.2 26.2s12.5 14.8 22.2 14.8l119.5 0z"/>
											</svg>
                                                </Pressable>
                                            </View>

                                        </View>

                                    </View>

                                    {/* Stop-rover button. */}
                                    <Pressable
                                        style={[ControllerStyle.icons, {maxHeight: '30%'}]}
                                        onPress={() => {
                                            this.sendOperation(Robot.WHEELS, WheelOperation.STOP);
                                            this.setState({ ls: 0, rs: 0 });
                                        }}
                                    >
                                    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" height="100%" width="100%">
                                        <path fill="#ffffff" d="M464 256A208 208 0 1 0 48 256a208 208 0 1 0 416 0zM0 256a256 256 0 1 1 512 0A256 256 0 1 1 0 256zm192-96l128 0c17.7 0 32 14.3 32 32l0 128c0 17.7-14.3 32-32 32l-128 0c-17.7 0-32-14.3-32-32l0-128c0-17.7 14.3-32 32-32z"/>\
                                    </svg>
                                    </Pressable>
                                    <View style={{ flexDirection: 'column' }}>
                                        {/*Right Top Buttons*/}
                                        <View style={{ flexDirection: 'row', gap: '15px' }}>

                                            {/* Raise right-shoulder button. */}
                                            <View style={{width: 30, height: 30}}>
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
                                            </View>

                                            {/* Lower right-shoulder button. */}
                                            <View style={{width: 30, height: 30}}>
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
                                            </View>
                                        </View>
                                        <View style={{ flexDirection: 'row', gap: '15px' }}>

                                            {/* Dig right-bucket button. */}
                                            <View style={{width: 30, height: 30}}>
                                                <Pressable
                                                    onPressIn={() => {
                                                        this.sendOperation(
                                                            Robot.FRONTDRUM,
                                                            DrumOperation.DIG
                                                        );
                                                    }}
                                                    onPressOut={() => {
                                                        this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                                    }}
                                                >
													<svg xmlns="http://www.w3.org/2000/svg"
											viewBox="0 0 512 512"
											style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path fill="#ffffff" path d="M48.5 224L40 224c-13.3 0-24-10.7-24-24L16 72c0-9.7 5.8-18.5 14.8-22.2s19.3-1.7 26.2 5.2L98.6 96.6c87.6-86.5 228.7-86.2 315.8 1c87.5 87.5 87.5 229.3 0 316.8s-229.3 87.5-316.8 0c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0c62.5 62.5 163.8 62.5 226.3 0s62.5-163.8 0-226.3c-62.2-62.2-162.7-62.5-225.3-1L185 183c6.9 6.9 8.9 17.2 5.2 26.2s-12.5 14.8-22.2 14.8L48.5 224z"/>
											</svg>
                                                </Pressable>
                                            </View>

                                            {/* Dump right-bucket button. */}
                                            <View style={{width: 30, height: 30}}>
                                                <Pressable
                                                    onPressIn={() => {
                                                        this.sendOperation(
                                                            Robot.FRONTDRUM,
                                                            DrumOperation.DUMP
                                                        );
                                                    }}
                                                    onPressOut={() => {
                                                        this.sendOperation(Robot.FRONTDRUM, DrumOperation.STOP);
                                                    }}
                                                >
											<svg xmlns="http://www.w3.org/2000/svg"
												viewBox="0 0 512 512"
												style={{ width: '100%', height: '100%', display: 'inline'}}
											>
												<path fill="#ffffff"  path d="M463.5 224l8.5 0c13.3 0 24-10.7 24-24l0-128c0-9.7-5.8-18.5-14.8-22.2s-19.3-1.7-26.2 5.2L413.4 96.6c-87.6-86.5-228.7-86.2-315.8 1c-87.5 87.5-87.5 229.3 0 316.8s229.3 87.5 316.8 0c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0c-62.5 62.5-163.8 62.5-226.3 0s-62.5-163.8 0-226.3c62.2-62.2 162.7-62.5 225.3-1L327 183c-6.9 6.9-8.9 17.2-5.2 26.2s12.5 14.8 22.2 14.8l119.5 0z"/>
											</svg>
                                                </Pressable>
                                            </View>
                                        </View>

                                    </View>
                                </View>
                                {/* Bottom row of controls. */}
                                <View style={{ flexDirection: "row", justifyContent: 'space-between' }}>


                                </View>
                            </View>
                        </View>



                        {/* Left button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
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
                            resizeMode="contain"
                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 512" height="45%" width="45%">
									<path fill="#ffffff" d="M9.4 233.4c-12.5 12.5-12.5 32.8 0 45.3l192 192c12.5 12.5 32.8 12.5 45.3 0s12.5-32.8 0-45.3L77.3 256 246.6 86.6c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0l-192 192z"/>
							</svg>

                            <View style={ControllerStyle.circleOverlayDown}>
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

                        {/* Right button. */}
                        <Pressable style={VideoStyle.upAndDownDPad}
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
                            resizeMode="contain"

                        >
                            <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 512" height="50%" width="50%">
									<path fill="#ffffff" d="M310.6 233.4c12.5 12.5 12.5 32.8 0 45.3l-192 192c-12.5 12.5-32.8 12.5-45.3 0s-12.5-32.8 0-45.3L242.7 256 73.4 86.6c-12.5-12.5-12.5-32.8 0-45.3s32.8-12.5 45.3 0l192 192z"/>
							</svg>
                            <View style={ControllerStyle.circleOverlayDown}>
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
                </View>
            </View>
        );
    }
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        backgroundColor: '#5D6061',
        alignItems: 'center',
        justifyContent: 'center',
    },
    webViewContainer: {
        flex: 1,
        alignContent: 'center',
        width: "100%",
        height: '100%',
    },
    webView: {
        objectFit: 'fill',
        width: '100vw',
        height: '100vh',
        display: 'block',
        top: 0,
        left: 0,
        margin: 0,
        padding: 0,
    },
    buttonContainer: {
        position: 'absolute',
        bottom: 16,
        flexDirection: 'row',
        justifyContent: 'space-evenly',
        alignContent: 'center',
        zIndex: 10,
		height: 225,
        flex: 2,
        elevation: 3,
        opacity: 0.8,
        width: '100%',
        paddingHorizontal: 70,

    },
    buttonText: {
        color: 'white',
        fontWeight: 'bold',
    },
    overlayedMenu: {
        flex: 1,
        position: 'absolute',
        top: 5,
        flexDirection: 'row',
        width: '100%',
        height: '10%',
        justifyContent: 'space-around',
        zIndex: 10,
    },
});
