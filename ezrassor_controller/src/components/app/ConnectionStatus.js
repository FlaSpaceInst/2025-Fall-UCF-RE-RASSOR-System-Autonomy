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
                    text: 'Connection to rover failed',
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
				<View flex={1} >
					<StatusBar backgroundColor="#2E3030" barStyle="dark-content" />

					<View style={ControllerStyle.screenLayout} >

						{/* Title container. */}
						<View style={[ControllerStyle.title]}>
							<Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.titleText]}>
								RE-RASSOR Connect
							</Text>
						</View>

						{/* Body container. */}
						<View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>

							{/* FSI logo. */}
							<Image source={logo} style={ControllerStyle.fsiLogo} />

							{/* Inner-body container. */}
							<View  width='60%' justifyContent='space-between' style={ControllerStyle.containerTwo}>
								<Image source={this.state.screens[this.state.screen].img} 
								resizeMode="contain"  style={{ width: '15%', height: '20%', tintColor: 'rgba(255, 255, 255, 1)'}}
								/>

								{/* Message to user. */}
								<Text
									adjustsFontSizeToFit={true}
									numberOfLines={2}
									style={{
										display: 'flex',
										alignSelf: 'center',
										fontfamily: 'Comic Sans MS',
										margin: 10,
										fontSize: normalize(30),
										textAlign: 'center',
										color: 'white',
										margin: '5%'
									}}
								>
									{this.state.screens[this.state.screen].text}
								</Text>

								{/* Retry/continue button. */}
								<Pressable
									adjustsFontSizeToFit={true}
									numberOfLines={1}
									title={this.state.screens[this.state.screen].buttonTxt}
									backgroundColor='#3F4142'
									style={[ControllerStyle.statusButton, {bottom: 0, width: '30%', height: '25%'}]}
									onPress={() => {
										this.redirect();
									}}
								>
									<Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.statusButtonText, {fontSize: normalize(28), padding: '6.5'}]}>
										{this.state.screens[this.state.screen].buttonTxt}
									</Text>
								</Pressable>

							</View>

							{/* Help button. */}
							<Pressable
								style={[ControllerStyle.buttonContainer, {width: '10%', maxWidth: 80, height: '15%'}]}
								onPress={() => {
									this.props.navigation.navigate("Connection Help Screen");
								}}
							>
								<Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.buttonText, {padding: 6.5}]}>
									Help
								</Text>
							</Pressable>

						</View>

					</View>
				</View>
			</View>
        );
    }
}
