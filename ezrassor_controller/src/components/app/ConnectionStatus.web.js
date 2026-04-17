import React from 'react';
import {
    Text,
    View,
    Image,
    StatusBar,
    Pressable
} from 'react-native';
import EZRASSOR from '../../../src/api/ezrassor-service';
import ControllerStyle from '../../../src/styles/controller';
import * as Font from 'expo-font';
import logo from '../../../assets/fsiLogo.png';
import checkmark from '../../../assets/checkmarktransparent.png';
import notconnectedtransparent from '../../../assets/notconnectedtransparent.png';
import wifi from '../../../assets/wifi.png';
import wifidisconnect from '../../../assets/wifidisconnect.png';
import * as SplashScreen from 'expo-splash-screen';
import { normalize } from '../../functionality/display';
/**
 * 
 * Modal-like React component displayed after user attempts to connect to an IP.
 * 
 * Multipurpose: Used both for successes and failures to connect.
 */
export default class ConnectionStatus extends React.Component {

    constructor(props) {
        SplashScreen.hideAsync();
        super(props);

        this.state = {
            screen: '',
            isLoading: true,
            ip: '',
            screens: {
                'roverConnected': {
                    img: checkmark,
                    text: 'SUCCESSFULLY CONNECTED TO ROVER',
                    buttonTxt: 'OK',
                    redirect: 'Controller Screen'
                },
                'roverDisconnected': {
                    img: notconnectedtransparent,
                    text: 'CONNECTION TO ROVER FAILED',
                    buttonTxt: 'RETRY',
                    redirect: 'IPConnect Screen'
                },
                'wifiConnected': {
                    img: wifi,
                    text: 'WIFI SUCCESSFULLY RECONNECTED',
                    buttonTxt: 'OK',
                    redirect: 'Controller Screen'
                },
                'wifiDisconnected': {
                    img: wifidisconnect,
                    text: 'WIFI DISCONNECTED',
                    buttonTxt: 'RETRY',
                    function: ''
                }
            }
        };

        this.EZRASSOR = new EZRASSOR(this.state.ip);
    }

    async componentDidMount() {

        this.setState({ screen: this.props.route.params.screen });
        this.setState({ isLoading: false });

        this._onfocus = this.props.navigation.addListener('focus', () => {
            this.setState({ screen: this.props.route.params.screen });
        });

        //Maybe this works?
        //this.sendOperation(Robot.ALL, Operation.STOP);
    }

    async componentWillUnmount() {
        this._onfocus();
    }

    redirect() {
        const currScreen = this.state.screens[this.state.screen];
        if (currScreen.hasOwnProperty('redirect')) {
            this.props.navigation.navigate(currScreen.redirect);
        }
    }

    render() {
        // I.e., don't do full render if font is still loading...
        if (this.state.isLoading) {
            return <View style={{ backgroundColor: '#5D6061' }} />;
        }

        return (
            <View style={{ height: '100%', flex: 1 }} >
                <StatusBar backgroundColor="#2E3030" barStyle="dark-content" />

                <View style={ControllerStyle.screenLayout} >

                    {/* Title container. */}
                    <View style={[ControllerStyle.title, { marginTop: '15%' }]}>
                        <Text adjustsFontSizeToFit={false} numberOfLines={1} style={[ControllerStyle.textMedium, { justifyContent: 'center', fontSize: '3.5vw' }]}>
                            RE-RASSOR Connect
                        </Text>
                    </View>

                    {/* Body container. */}
                    <View style={{ flexDirection: 'column', justifyContent: 'space-between' }}>
                        {/* Inner-body container. */}
                        <View backgroundColor='#FFFFFF' width='80%' justifyContent='space-between' style={ControllerStyle.containerTwo}>
                        <Image 
                            source={this.state.screens[this.state.screen].img} 
                            style={{ width: '15%', height: '15%', tintColor: 'rgba(255, 255, 255, 1)', margin: '10%' }} 
                            resizeMode="contain" 
                        />
                            {/* Message to user. */}
                            <Text
                                adjustsFontSizeToFit={true}
                                numberOfLines={2}
                                style={{
                                    display: 'flex',
                                    alignSelf: 'center',
                                    fontfamily: 'Comic Sans MS',
                                    overflow: 'visible',
                                    marginBottom: '10%',
                                    fontSize: '2.5vw',
                                    textAlign: 'center',
                                    color: 'white'
                                }}
                            >
                                {this.state.screens[this.state.screen].text}
                            </Text>

                            {/* Retry/continue button. */}
                            <Pressable
                                adjustsFontSizeToFit={false}
                                numberOfLines={1}
                                title={this.state.screens[this.state.screen].buttonTxt}
                                backgroundColor='#3F4142'
                                style={[ControllerStyle.statusButton, { maxHeight: '17%' }]}
                                onPress={() => {
                                    this.redirect();
                                }}
                            >
                                <Text adjustsFontSizeToFit={false} numberOfLines={1} style={[ControllerStyle.statusButtonText, { fontSize: '2vw' }]}>
                                    {this.state.screens[this.state.screen].buttonTxt}
                                </Text>
                            </Pressable>

                        </View>

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
                            style={[ControllerStyle.buttonContainer, { height: '16%', alignSelf: 'auto', marginRight: '5%' }]}
                            onPress={() => {
                                this.props.navigation.navigate('Connection Help Screen');
                            }}
                        >
                            <Text style={[ControllerStyle.buttonText, { fontSize: '2vw' }]}>
                                Help
                            </Text>
                        </Pressable>

                    </View>

                </View>
            </View>
        );
    }
}
