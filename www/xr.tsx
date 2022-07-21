import { fromOffers } from "./modules/rtc";
import { pullOffers, postAnswer } from "./modules/server";
import React from 'react';
import { createRoot } from 'react-dom/client';
import styled, { createGlobalStyle } from 'styled-components';
import Viewport from './view/Viewport';



const GlobalStyle = createGlobalStyle`
    html, body, #robotic_avatar {
        margin: 0;
    }
`

const View = styled(Viewport)`
    position: absolute;
    width: 100vw;
    height: 100vh;
`

const Start = styled.button`
    position: absolute;
    width: 200px;
    height: 200px;
`

const HiddenVideo = styled.video`
    display: none;
`

interface State {
    left?: MediaStream,
    right?: MediaStream,
}

class RoboticAvatar extends React.Component<{}, State> {
    constructor(props){
        super(props);       
        this.state = {}
    }

    render(): React.ReactNode {
        return <>
            <GlobalStyle/>
            <View left={this.state.left} right={this.state.right}/>
            <Start id="start" onClick={this.startHandler}>START</Start>
        </>
    }

    startHandler = async () => {
        try {
            document.getElementById("start")!.hidden = true;
            let offers = await pullOffers();
            console.log(offers);
            let con = await fromOffers(offers);
            let answer = await con.createAnswers();
            console.log(answer);
            await postAnswer(answer);
            let streams = con.getStreams();
            this.setState(streams);
        } catch (err) {
            console.log(err.name + ": " + err.message);
        }
    } 
}


const react_root = createRoot(document.getElementById("robotic_avatar")!);
react_root.render(<RoboticAvatar/>);

