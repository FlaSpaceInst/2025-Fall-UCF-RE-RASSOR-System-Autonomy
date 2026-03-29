import React, { useState } from 'react';
import Modal from "react-native-modal";
import EZRASSOR from '../../../src/api/ezrassor-service-paver-arm'
import ControllerStyle from '../../../src/styles/controller';
import { Robot, Operation } from '../../../src/enumerations/robot-commands-paver-arm';
import FadeInView from "../../../src/components/app/FadeInView";
import { isIpReachable } from '../../functionality/connection';
import {
    Text,
    View,
    Image,
    TouchableHighlight,
    Pressable,
    StatusBar,
    KeyboardAvoidingView,
    TextInput,
} from 'react-native';
import { FontAwesome, MaterialCommunityIcons } from '@expo/vector-icons';
import * as Font from 'expo-font';

const CONNECTION_POLLING_INTERVAL = 2000;
const CONNECTION_POLLING_TIMEOUT = 4000;
const CONNECTION_POLLING_VERBOSE = false;



export default class ControllerScreen extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            autonomyModalVisible: false,
            infoModalVisible: false,
            devModalVisible: false,
            ipModal: false,
            xyModal: false,
            isLoading: true,
            control: 0,
            xy: '0,0',
            ip: '',
            isDefault: true,
            isPlateSelected: false,
            isShoulderSelected: false,
            isForearmSelected: false,
            isWristSelected: false,
        };

        this.EZRASSOR = new EZRASSOR(this.state.ip);
    }

    async componentDidMount() {
        await this.getIpFromStorage();
        this.state.ip = ''
        //this.state.ip = this.props.route.params.currentIp;
        // important line: sets the IP address for current EZRASSOR instance
        this.EZRASSOR.host = this.state.ip;

        this.setState({ isLoading: false });

        this._unsubscribe = this.props.navigation.addListener('focus', async () => {
            this.getIpFromStorage();
        });

        // Set up the connection polling logic.
        let pollCount = 0;
        this.connectionPoller = setInterval(async () => {
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
        console.log(`Polling every ${pollInterval}s with a ${pollTimeout}s timeout.`);
    }

    async componentWillUnmount() {
        clearInterval(this.connectionPoller);

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
    // copy paste ends here

    setAutonomyModalVisible(visible) {
        this.setState({ autonomyModalVisible: visible });
    }

    setInfoModalVisible(visible) {
        this.setState({ infoModalVisible: visible });
    }

    setBugModalVisible(visible) {
        this.setState({ bugModalVisible: visible });
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

    changeIP(text) {
        this.setState({ ip: text }, () => {
            this.EZRASSOR.host = this.state.ip;
        });
    }

    // Update animation frame before processing click so that opacity can change on click 
    sendOperation(part, operation) {
        requestAnimationFrame(() => {
            this.EZRASSOR.executeRobotCommand(part, operation);
        });
    }

    renderDefaultArm = () => {
        if (this.state.isDefault) {
            return (
                <Image 
                    source={require('../../../assets/PaverArmDefault.png')} 
                    style={{ width: "100%", height: "100%" }} 
                    resizeMode="contain" // Set resizeMode as a prop
                />
            );
        } else {
            return null;
        }
    };

    renderPlateArm = () => {
        if (this.state.isPlateSelected) {
            return (
                <Image 
                    source={require('../../../assets/PaverArmBase.png')} 
                    style={{ width: "100%", height: "100%" }} 
                    resizeMode="contain" // Set resizeMode as a prop
                />
            );
        
        } else {
            return null;
        }
    };

    renderShoulderArm = () => {
        if (this.state.isShoulderSelected) {
            return (
                <Image 
                    source={require('../../../assets/PaverArmShoulder.png')} 
                    style={{ width: "100%", height: "100%" }} 
                    resizeMode="contain" // Set resizeMode as a prop
                />
            );            
        } else {
            return null;
        }
    };

    renderForearmArm = () => {
        if (this.state.isForearmSelected) {
            return (
                <Image 
                    source={require('../../../assets/PaverArmForearm.png')} 
                    style={{ width: "100%", height: "100%" }} 
                    resizeMode="contain" // Set resizeMode as a prop
                />
            );
        
        } else {
            return null;
        }
    };

    renderWristArm = () => {
        if (this.state.isWristSelected) {
            return (
                <Image 
                    source={require('../../../assets/PaverArmWrist.png')} 
                    style={{ width: "100%", height: "100%" }} 
                    resizeMode="contain" // Set resizeMode as a prop
                />
            );            
        } else {
            return null;
        }
    };

    render() {

        // Loading font
        if (this.state.isLoading) {
            return (
                <View style={{ flex: 1, backgroundColor: '#5D6061' }} />
            );
        }

        return (
            <View style={ControllerStyle.container}>
                <StatusBar hidden />

                {/* Info Popup Modal*/}
                <Modal
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.infoModalVisible}
                    onSwipeComplete={() => this.setInfoModalVisible(false)}
                    swipeDirection='down'
                    onRequestClose={() => this.setInfoModalVisible(!this.state.infoModalVisible)}>

                    <TouchableHighlight style={{ justifyContent: 'center' }}>
                        <View>
                            <View style={{ flexDirection: 'row', justifyContent: 'center', marginBottom: 25 }}>
                                <Text style={ControllerStyle.textLarge}>Controller Help</Text>
                            </View>

                            <View style={{ flexDirection: 'row', justifyContent: 'center', marginHorizontal: "0.5%" }}>
                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <FontAwesome style={{ textAlign: 'center' }} name="wifi" size={30} color='#fff' />
                                    <Text style={ControllerStyle.textSmall}>
                                        Connects to the server with input string: {'\n'}
                                        IP : PORT
                                    </Text>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <FontAwesome style={{ textAlign: 'center' }} name="arrow-left" size={30} color='#fff' />
                                    <Text style={ControllerStyle.textSmall}>
                                        Back to homescreen
                                    </Text>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <FontAwesome style={{ textAlign: 'center' }} name="stop-circle-o" size={30} color='#fff' />
                                    <Text style={ControllerStyle.textSmall}>
                                        Emergency stop for manual movements
                                    </Text>
                                </View>

                                {/* Vertical separating bar. */}
                                <View style={{ flex: 0.01, borderRadius: 2, backgroundColor: "#2e3030", marginHorizontal: "0.5%" }}></View>

                                <View style={{ flex: 1, justifyContent: 'space-evenly' }} >
                                    <MaterialCommunityIcons style={{ textAlign: 'center' }} name="robot" size={32} color='#fff' />
                                    <Text style={ControllerStyle.textSmall}>
                                        Opens autonomy functions
                                    </Text>
                                </View>
                            </View>
                        </View>
                    </TouchableHighlight>
                </Modal>

                {/* Autonomy Popup Modal*/}
                <Modal
                    style={ControllerStyle.modalViewContainer}
                    isVisible={this.state.autonomyModalVisible}
                    onSwipeComplete={() => this.setAutonomyModalVisible(false)}
                    swipeDirection='down'
                    onRequestClose={() => this.setAutonomyModalVisible(!this.state.autonomyModalVisible)}>

                    <TouchableHighlight style={{ flex: 1, marginHorizontal: 15, justifyContent: 'center' }}>
                        <View>
                            <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
                                <Text style={ControllerStyle.textLarge}>Activate Autonomous Arm Function(s)</Text>
                            </View>
                            <View style={{ flexDirection: 'row', marginVertical: 15, justifyContent: 'center' }}>
                                <Pressable style={ControllerStyle.modalButton} onPress={() => { this.sendOperation(Robot.AUTONOMY, Operation.PICKUP) }}>
                                    <Text style={ControllerStyle.textSmall}>Pick Up Paver</Text>
                                </Pressable>
                                <Pressable style={ControllerStyle.modalButton} onPress={() => { this.sendOperation(Robot.AUTONOMY, Operation.PLACE) }}>
                                    <Text style={ControllerStyle.textSmall}>Place Paver</Text>
                                </Pressable>
                                <Pressable style={ControllerStyle.modalButton} onPress={() => { this.sendOperation(Robot.AUTONOMY, Operation.HOME) }}>
                                    <Text style={ControllerStyle.textSmall}>Return Home</Text>
                                </Pressable>
                            </View>
                        </View>
                    </TouchableHighlight>
                </Modal>

                <FadeInView style={ControllerStyle.headerContainer}>
                    <View style={{ flexDirection: 'row', gap: "15px" }}>
                        {/* Info button */}
                        <Pressable style={ControllerStyle.icons} onPress={() => this.setInfoModalVisible(true)}>
                            <FontAwesome
                                name="info-circle"
                                size={50} color='#fff' />
                        </Pressable>

                        {/* Set-IP button. */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("IPConnect Screen")}>
                            <FontAwesome
                                name="wifi"
                                size={50} color="#fff" />
                        </Pressable>
                    </View>

                    <Text style={ControllerStyle.textMedium}>RE-RASSOR Paver Arm Controller</Text>

                    <View style={{ flexDirection: 'row', gap: "15px" }}>
                        {/* BACK button to go back to the controller screen */}
                        <Pressable
                            style={ControllerStyle.icons}
                            onPress={() => this.props.navigation.replace("Controller Screen", { currentIp: this.state.ip })}
                        >
                            <FontAwesome
                                name="arrow-left"
                                size={50}
                                color="#fff" />
                        </Pressable>

                        {/* Stop button */}
                        <Pressable style={ControllerStyle.icons} onPress={() => { this.sendOperation(Robot.ALL, Operation.STOP) }}>
                            <FontAwesome name="stop-circle-o"
                                size={50}
                                color='#fff' />
                        </Pressable>

                        {/* Autonomy button for arm */}
                        <Pressable style={ControllerStyle.icons} onPress={() => { this.setAutonomyModalVisible(true) }}>
                            <MaterialCommunityIcons
                                name="robot"
                                size={50}
                                color='#fff' />
                        </Pressable>
                    </View>
                </FadeInView>

                {/* Control pad:  */}
                <FadeInView style={ControllerStyle.buttonLayoutContainer}>
                    <View style={ControllerStyle.ArmContainer}>
                        <View style={{ flex: 2, flexDirection: 'row' }}>

                            {/* Left controls */}
                            <View style={{ flex: 2, flexDirection: 'column' }}>
                                <View style={{ flex: 2, flexDirection: 'column' }}>
                                    <View style={ControllerStyle.PaverArmBaseButton}>
                                        <Text style={ControllerStyle.buttonTextCenter}>Shoulder</Text>
                                        {/* Shoulder controls */}
                                        <View style={{ flex: 1, flexDirection: 'row', gap: 15, justifyContent: 'space-evenly', alignItems: 'center' }}>
                                            {/* Shoulder Up */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.SHOULDER, Operation.POSITIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: false });
                                                    this.setState({ isShoulderSelected: true });
                                                    this.setState({ isForearmSelected: false });
                                                    this.setState({ isWristSelected: false });
                                                }}
                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-up" size={50} color='#fff' />
                                            </Pressable>

                                            {/* Shoulder Down */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.SHOULDER, Operation.NEGATIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: false });
                                                    this.setState({ isShoulderSelected: true });
                                                    this.setState({ isForearmSelected: false });
                                                    this.setState({ isWristSelected: false })
                                                }}
                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-down" size={50} color='#fff' />
                                            </Pressable>
                                        </View>
                                    </View>

                                    {/* Plate  */}
                                    <View style={ControllerStyle.PaverArmBaseButton}>
                                        <Text style={ControllerStyle.buttonTextCenter}>Plate</Text>
                                        {/* Plate controls */}
                                        <View style={{ flex: 1, flexDirection: 'row', gap: 15, justifyContent: 'space-evenly', alignItems: 'center' }}>
                                            {/* Plate left */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.PLATE, Operation.POSITIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: true });
                                                    this.setState({ isShoulderSelected: false });
                                                    this.setState({ isForearmSelected: false });
                                                    this.setState({ isWristSelected: false })
                                                }}

                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="rotate-left" size={50} color='#fff' />
                                            </Pressable>

                                            {/* Plate Right */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.PLATE, Operation.NEGATIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: true });
                                                    this.setState({ isShoulderSelected: false });
                                                    this.setState({ isForearmSelected: false });
                                                    this.setState({ isWristSelected: false })
                                                }}

                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="rotate-right" size={50} color='#fff' />
                                            </Pressable>
                                        </View>
                                    </View>

                                </View>
                            </View>

                            {/* Center Arm */}
                            {<View style={ControllerStyle.PaverArmBackground}>
                                {this.renderDefaultArm()}
                                {this.renderPlateArm()}
                                {this.renderShoulderArm()}
                                {this.renderForearmArm()}
                                {this.renderWristArm()}
                            </View>}

                            {/* Right controls */}
                            <View style={{ flex: 2, flexDirection: 'column' }}>
                                <View style={{ flex: 2, flexDirection: 'column' }}>

                                    {/* Wrist */}
                                    <View style={ControllerStyle.PaverArmBaseButton}>
                                        <Text style={ControllerStyle.buttonTextCenter}>Wrist</Text>
                                        {/* Wrist controls */}
                                        <View style={{ flex: 1, flexDirection: 'row', gap: 15, justifyContent: 'space-evenly', alignItems: 'center' }}>
                                            {/* Wrist Up */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.WRIST, Operation.POSITIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: false });
                                                    this.setState({ isShoulderSelected: false });
                                                    this.setState({ isForearmSelected: false });
                                                    this.setState({ isWristSelected: true })
                                                }}

                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-up" size={50} color='#fff' />
                                            </Pressable>
                                            {/* Wrist Down */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.WRIST, Operation.NEGATIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: false });
                                                    this.setState({ isShoulderSelected: false });
                                                    this.setState({ isForearmSelected: false });
                                                    this.setState({ isWristSelected: true })
                                                }}

                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-down" size={50} color='#fff' />
                                            </Pressable>
                                        </View>
                                    </View>

                                    {/* Forearm  */}
                                    <View style={ControllerStyle.PaverArmBaseButton}>
                                        <Text style={ControllerStyle.buttonTextCenter}>Elbow</Text>
                                        {/* Forearm controls */}
                                        <View style={{ flex: 1, flexDirection: 'row', gap: 15, justifyContent: 'space-evenly', alignItems: 'center' }}>
                                            {/* Forearm Up */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.FOREARM, Operation.POSITIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: false });
                                                    this.setState({ isShoulderSelected: false });
                                                    this.setState({ isForearmSelected: true });
                                                    this.setState({ isWristSelected: false })
                                                }}


                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-up" size={50} color='#fff' />
                                            </Pressable>

                                            {/* Forearm Down */}
                                            <Pressable
                                                style={ControllerStyle.buttonBackground}
                                                onPressIn={() => {
                                                    this.sendOperation(Robot.FOREARM, Operation.NEGATIVE);
                                                    this.setState({ isDefault: false });
                                                    this.setState({ isPlateSelected: false });
                                                    this.setState({ isShoulderSelected: false });
                                                    this.setState({ isForearmSelected: true });
                                                    this.setState({ isWristSelected: false })
                                                }}

                                            >
                                                <FontAwesome tyle={ControllerStyle.buttonImage} name="chevron-down" size={50} color='#fff' />
                                            </Pressable>
                                        </View>
                                    </View>

                                </View >
                            </View >
                        </View >
                    </View >
                </FadeInView >
            </View >
        );
    }
} 
