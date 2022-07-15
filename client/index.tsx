import { createRoot } from 'react-dom/client';
import RoboticAvatarView from "./views/RoboticAvatar";
import RoboticAvatarModel from "./models/robotic-avatar";
import { Webcams } from './services/webcams';
import React from 'react';

import * as client from './wasm/';

function main(){
    // Rust test:
    client.default().then(() => {
        client.greet()
    })

    // Load services.
    const webcams = new Webcams();

    // Load Models
    const model = new RoboticAvatarModel(webcams);

    // Setup UI / View layer
    const app = document.getElementById("robotic_avatar");
    const react_root = createRoot(document.getElementById("robotic_avatar")!);
    react_root.render(<RoboticAvatarView model={model}/>);
}

main();