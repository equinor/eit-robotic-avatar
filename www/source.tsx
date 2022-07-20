import React from "react";
import { loadCams } from "./modules/cameras";
import { fromStreams } from "./modules/rtc";
import { postOffers, pullAnswer } from "./modules/server";
import { createRoot } from 'react-dom/client';
import styled, { createGlobalStyle } from "styled-components";
import Viewport from "./view/Viewport";

const GlobalStyle = createGlobalStyle`
    html, body, #robotic_avatar {
        margin: 0;
        height: 100%;
    }
`

const View = styled(Viewport)`
    height: 100%;
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
        </>
    }

    componentDidMount() {
        const inner = async () => {
            let cams = await loadCams();
            this.setState(cams);
            let con = await fromStreams(cams);
            let offers = await con.createOffers();
            console.log(offers);
            await postOffers(offers);
            let answer = await pullAnswer();
            console.log(answer);
            await con.setAnswers(answer);
        }
        
        inner().catch(console.error);
    }
}

const react_root = createRoot(document.getElementById("robotic_avatar")!);
react_root.render(<RoboticAvatar/>);
