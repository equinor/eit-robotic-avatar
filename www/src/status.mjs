//@ts-check

export default class Status {
    /**
     * @param {HTMLElement} elem
     */
    constructor(elem){
        this.elem = elem;
    }

    /**
     * @param {string} message
     */
    status(message) {
        this.elem.innerText = message;
        this.elem.className = ""
    }

    /**
     * @param {string} message
     */
    warn(message) {
        this.elem.innerText = message;
        this.elem.className = "warn";
    }

    /**
     * @param {string} message
     */
    error(message) {
        this.elem.innerText = message;
        this.elem.className = "error";
    }
}