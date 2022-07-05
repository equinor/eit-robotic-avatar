import { createRoot } from 'react-dom/client';
import RoboticAvatarView from "./views/RoboticAvatar";
import RoboticAvatarModel from "./models/robotic-avatar";
import Webcams from './services/webcams';

function main(){
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