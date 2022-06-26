//@ts-check
import { loadCams } from "./modules/cameras.mjs";

async function main() {
    try {
        await loadCams();
    } catch(e) {
        console.log(e.name + ": " + e.message);
    }
}

await main();