import React from 'react';
import {
    Text,
    View,
    Image,
    StatusBar,
    TextInput,
    Pressable,
    Modal,
    FlatList
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import ControllerStyle from '../../../src/styles/controller';
import * as Font from 'expo-font';
import logo from '../../../assets/fsiLogo.png';
import arrowright from '../../../assets/arrowri.png';
// import wifi from '../../../assets/wifi.png';
import { KeyboardAwareScrollView } from 'react-native-keyboard-aware-scroll-view';
import { isIpReachable } from '../../functionality/connection';
import { normalize } from '../../functionality/display';
import LoadingDots from './LoadingDots';

const DEFAULT_IP = '192.168.1.2:8080';

/**
 * React component for the connect-to-ip screen.
 */
export default class IPConnect extends React.Component {

    constructor(props) {
        super(props);

        this.state = {
            isLoading: true,
            ip: '',
            showModal: false,
            modalVisible: false,
            foundPotatoes: [],
            scanComplete: false,
          };
          
        

        this.animation = React.createRef(null);
    }

    async componentDidMount() {
        this.animation = React.createRef(null);
        this.animation.current?.reset();

        this.setState({ isLoading: false });
        this.getIpFromStorage();

        this._unsubscribe = this.props.navigation.addListener('focus', () => {
            this.getIpFromStorage();
            this.animation.current?.reset();
        });

        this._blur = this.props.navigation.addListener('blur', () => {
            this.animation.current?.reset();
        });

        if (window.api?.onPotatoFound) {
            console.log('✅ window.api.onPotatoFound exists');
            window.api.onPotatoFound((potato) => {
                console.log('🥔 Found Potato:', potato);
                this.setState(prev => ({
                    foundPotatoes: [...prev.foundPotatoes, potato]
                }));
            });
        } else {
            console.warn('❌ window.api.onPotatoFound is undefined');
        }
        
        
    }
          
    findPotatoes = () => {
        this.setState({
            modalVisible: true,
            foundPotatoes: [],
            scanComplete: false,
        });
    
        if (window.api?.scanPotatoes) {
            window.api.scanPotatoes();
    
            // Temporary timeout for "scan complete"
            setTimeout(() => {
                this.setState({ scanComplete: true });
            }, 5000); // or however long your scan takes
        } else {
            console.warn('Potato scanner API not available');
        }
    };
    
    async componentWillUnmount() {
        this.animation = React.createRef(null);
        this.animation.current?.reset();

        this._unsubscribe();
        this._blur();
    }

    /**
     * Set `this.state.ip` to the specified IP + port.
     * 
     * @param {string} ip IP + port.
     */
    changeIP(ip) {
        this.setState({ ip });
    }

    /**
     * Set `myIp` in storage to `this.state.ip`.
     */
    async setIpInStorage() {
        try {
            await AsyncStorage.setItem('myIp', this.state.ip);
        } catch (error) {
            // Error saving data. Log error, but do nothing otherwise.
            console.log(error);
        }
    }

    /**
     * Load `myIp` from storage into `this.state.ip`.
     */
    async getIpFromStorage() {
        try {
            const ip = await AsyncStorage.getItem('myIp');

            if (this.state.ip == '') {
                this.setState({
                    ip: (ip == '') ? DEFAULT_IP : ip
                });
            }
        } catch (error) {
            // Error retrieving data. Do nothing.
        }
    }

    /**
     * Check if we can connect to the ip address at `this.state.ip`. Then redirect to either
     * a "can connect" or "cannot connect" screen.
     */
    async redirectBasedOnReachability() {
        const timeoutTime = 5000;

        console.log('In redirectBasedOnReachability with IP: ' + this.state.ip);

        if (await isIpReachable(this.state.ip, timeoutTime)) {
            this.setIpInStorage();
            this.props.navigation.navigate('Connection Status Screen', { screen: 'roverConnected' });
        } else {
            this.props.navigation.navigate('Connection Status Screen', { screen: 'roverDisconnected' });
        }
    }
    
    // findPotatoes = () => {
    //     if (window.api?.scanPotatoes) {
    //       window.api.scanPotatoes(); // call exposed function
    //     } else {
    //       console.warn('Potato scanner API not available');
    //     }
    // };
    
    

    render() {
        // I.e., don't do full render if font is still loading...
        if (this.state.isLoading) {
            return <View style={{ flex: 1, backgroundColor: '#5D6061' }} />;
        }

        return (
            <KeyboardAwareScrollView contentContainerStyle={[ControllerStyle.keyboardAwareScrollView, {overflow: 'hidden'}]}>
                <View style={[ControllerStyle.screenLayout, { overflow: 'hidden' }]}>

                    <StatusBar backgroundColor="#2E3030" barStyle="dark-content" />

                    {/* Title container. */}
                    <View style={[ControllerStyle.title, { marginTop: '15%' }]}>
                        <Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.titleText, { fontSize: '3.5vw', justifyContent: 'center' }]}>
                            RE-RASSOR Connect
                        </Text>
                    </View>

                    {/* Body container. */}
                    <View style={{ flexDirection: 'column', justifyContent: 'space-between', marginTop: '2%', width: '70%' }}>

                        {/* Inner-body container. */}
                        <View style={[ControllerStyle.containerTwo, { backgroundColor: '#4a4d4e', width: '70%', height: '85%', padding: '5%' }]}>

                            {/* Message to user. */}
                            <Text
                                adjustsFontSizeToFit={true}
                                //numberOfLines={1}
                                style={{
                                    display: 'flex',
                                    alignSelf: 'center',
                                    textAlign: 'center',
                                    fontfamily: 'Comic Sans MS',
                                    margin: 10,
                                    fontSize: '2vw',
                                    color: '#fff',
                                    //paddingTop: 10, // Add padding to the top
                                    //paddingBottom: 10, // Add padding to the bottom
                                }}
                            >
                                Please enter the IP address of the RE-RASSOR cart:
                            </Text>

                            {/* Loading dots animation. */}
                            <View
                                style={[{ justifyContent: 'center', alignItems: 'center', position: 'absolute' }]}>
                                <LoadingDots ref={this.animation} autoPlay={false} />
                            </View>

                            {/* Text input for IP + port. */}
                            <TextInput
                                //fontSize={normalize(32, 1.8)}
                                style={[ControllerStyle.ipInputBox, { fontSize: '2vw' }]}
                                onChangeText={(text) => this.changeIP(text)}
                                value={this.state.ip}
                                marginVertical={8}
                                disableFullscreenUI={true}
                                selectionColor={'white'}
                                id='IP input form'
                            />
                            <View style={{ flexDirection: 'row', justifyContent: 'space-around', marginTop: '2%', width: '70%' }}>
                                
                                {/* Find Rovers button */}
                                <Pressable
                                    activeOpacity={0.95}
                                    style={[ControllerStyle.connectButton, { padding: '2%', height: 'auto', backgroundColor: 'white', flex: 1, marginRight: '1%' }]}
                                    onPress={() => this.findPotatoes()}
                                >
                                    <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
                                        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 640 512" height="60%" width="50%">
                                            <path fill="#000000" d="M54.2 202.9C123.2 136.7 216.8 96 320 96s196.8 40.7 265.8 106.9c12.8 12.2 33 11.8 45.2-.9s11.8-33-.9-45.2C549.7 79.5 440.4 32 320 32S90.3 79.5 9.8 156.7C-2.9 169-3.3 189.2 8.9 202s32.5 13.2 45.2 .9zM320 256c56.8 0 108.6 21.1 148.2 56c13.3 11.7 33.5 10.4 45.2-2.8s10.4-33.5-2.8-45.2C459.8 219.2 393 192 320 192s-139.8 27.2-190.5 72c-13.3 11.7-14.5 31.9-2.8 45.2s31.9 14.5 45.2 2.8c39.5-34.9 91.3-56 148.2-56zm64 160a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z"/>
                                        </svg>
                                    </div>
                                    <Text style={[ControllerStyle.connectButtonText, { fontSize: '1.5vw', fontWeight: '600' }]}>
                                        FIND ROVERS
                                    </Text>
                                    
                                </Pressable>

                                {/* Connect button */}
                                <Pressable
                                    activeOpacity={0.95}
                                    style={[ControllerStyle.connectButton, { padding: '2%', height: 'auto', backgroundColor: 'white', flex: 1, marginLeft: '1%' }]}
                                    onPress={() => {
                                        this.animation.current?.play();
                                        this.redirectBasedOnReachability();
                                    }}
                                >
                                    <Text style={[ControllerStyle.connectButtonText, { fontSize: '1.5vw', fontWeight: '600' }]}>
                                        CONNECT
                                    </Text>
                                    <Image source={arrowright} style={ControllerStyle.arrowRight} resizeMode="contain" />
                                </Pressable>
                            </View>


                        </View>


                        {/* Help button. */}

                    </View>
                    <View style={[{ flexDirection: 'row', justifyContent: 'space-between', width: '100%', height: '40%', marginTop: '2%' }]}>

                        {/* FSI logo. */}
                        <Image
                            source={logo}
                            style={[ControllerStyle.fsiLogo, { alignSelf: 'auto', height: '21%' }]}
                            resizeMode="contain"
                        />

                        <Pressable
                            activeOpacity={0.95}
                            style={[ControllerStyle.buttonContainer, { height: '16%', alignSelf: 'auto', marginRight: '5%', padding: '1.5%' }]}
                            onPress={() => {
                                this.props.navigation.navigate('Connection Help Screen');
                            }}
                        >
                            <Text style={[ControllerStyle.buttonText, { fontSize: '2vw', fontWeight: '500' }]}>
                                Help
                            </Text>
                        </Pressable>

                    </View>
                    <Modal
                        animationType="slide"
                        transparent={true}
                        visible={this.state.modalVisible}
                        onRequestClose={() => {
                            this.setState({ modalVisible: false });
                        }}
                        >
                        
                        <View style={{ flex: 1, justifyContent: 'center', alignItems: 'center', backgroundColor: 'rgba(0,0,0,0.5)' }}>
                            <View style={{ width: '65%', backgroundColor: 'grey', padding: 20, borderRadius: 10 }}>
                            <Text style={{ fontSize: 25, marginBottom: 10, color:'white' }}>Finding Rover IP Address:</Text>

                            {this.state.foundPotatoes.length > 0 ? (
                                <FlatList
                                data={this.state.foundPotatoes}
                                keyExtractor={(item, index) => index.toString()}
                                renderItem={({ item }) => (
                                    <Text style={{ fontSize: 15, color: 'white' }}>🖥️ {item.name} ({item.ip}:{item.port})</Text> // Note to self, Add text style later
                                )}
                                />
                            ) : this.state.scanComplete ? (
                                <Text style={{ fontSize: 15, color: 'white' }}>No potatoes found 😢</Text>
                            ) : (
                                <Text style={{ fontSize: 15, color: 'white' }}>Scanning...</Text>
                            )}

                            <Pressable
                                style={{ marginTop: 20, padding: 10, backgroundColor: '#545454', borderRadius: 5 }}
                                onPress={() => this.setState({ modalVisible: false })}
                            >
                                <Text style={{ color: 'white', textAlign: 'center' }}>Close</Text>
                            </Pressable>
                            </View>
                        </View>
                    </Modal>
                </View>
            </KeyboardAwareScrollView>
        );
    }
}
