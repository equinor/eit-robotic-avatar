import { createRoot } from 'react-dom/client';
import RoboticAvatarView from "./views/RoboticAvatar";
import RoboticAvatarModel from "./models/robot-avatar";

function main(){
    // Load services.

    // Load Models
    const model = new RoboticAvatarModel();

    // Setup UI / View layer
    const app = document.getElementById("robotic_avatar");
    const react_root = createRoot(document.getElementById("robotic_avatar")!);
    react_root.render(<RoboticAvatarView model={model}/>);
}

main();