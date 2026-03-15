import React from 'react';
import {
  Text,
  View,
  Image,
  StatusBar,
  Pressable
} from 'react-native';
import ControllerStyle from '../../../src/styles/controller';
import * as Font from 'expo-font';
import logo from '../../../assets/fsiLogo.png';
import * as SplashScreen from 'expo-splash-screen';
import { normalize } from '../../functionality/display';

const CONNECTION_TEXT = `1. Make sure both the phone and the rover are connected to the same WIFI.

2. Double check you are typing in the right IP address and port when trying to connect.

3. If steps 1 and 2 did not work, try restarting both the rover and the phone
(or restarting the application).`;

/**
 * React component used as a help screen for users at the <IPConnect> screen.
 */
export default class ConnectionHelp extends React.Component {

  constructor(props) {
    SplashScreen.hideAsync();
    super(props);

    this.state = {
      isLoading: true
    };

    this.connectionText = CONNECTION_TEXT;
  }

  async componentDidMount() {

    this.setState({ isLoading: false });
  }

  redirect() { }

  render() {
    // I.e., don't do full render if font is still loading...
    if (this.state.isLoading) {
      return <View style={{ backgroundColor: '#5D6061' }} />;
    }

    return (
      <View style={ControllerStyle.screenLayout} >

        <StatusBar backgroundColor="#2E3030" barStyle="dark-content" />

        {/* Title container. */}
        <View style={[ControllerStyle.title]}>
          <Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.titleText, {padding: 7}]}>
            RE-RASSOR Connection Help
          </Text>
        </View>

        {/* Body container. */}
        <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>

          {/* FSI logo. */}
          <Image source={logo} style={[ControllerStyle.fsiLogo]} />

          {/* Inner-body container. */}
          <View adjustsFontSizeToFit={true} style={[ControllerStyle.containerTwo, { width: '70%', flexWrap: 'nowrap' }]}>
            <Text
              adjustsFontSizeToFit={true}
              numberOfLines={10}
              style={{
                alignSelf: 'center',
				justifyContent: 'center',
                fontFamily: 'Comic Sans MS',
                margin: 5,
                fontSize: normalize(30, 1.5),
                color: '#fff',
				textAlign: 'center'
                }}
            >
              {this.connectionText}
            </Text>
          </View>

          {/* Back button. */}
          <Pressable
            style={[ControllerStyle.buttonContainer, {width: '10%', maxWidth: 80, height: '15%'}]}
            onPress={() => {
              this.props.navigation.goBack(null)
            }}
          >
            <Text adjustsFontSizeToFit={true} numberOfLines={1} style={[ControllerStyle.buttonText, {padding: 6.5}]}>
              Back
            </Text>
          </Pressable>

        </View>

      </View>
    );
  }
}
