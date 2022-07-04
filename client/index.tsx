import { createRoot } from 'react-dom/client';
import { RoboticAvatar } from "./RoboticAvatar";

const app = document.getElementById("robotic_avatar");
const root = createRoot(app!);
root.render(<RoboticAvatar />);