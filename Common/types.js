class Pose2D {
    constructor(x, y, yaw) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
    }
}
  
class Vector2D {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }
}

class Path{
    constructor() {
        // # course segment length  (negative value is backward segment)
        this.lengths = []
        // # course segment type char ("S": straight, "L": left, "R": right)
        this.ctypes = []
        this.L = 0.0 //Total lengths of the path
        this.x = [] //x positions
        this.y = [] //y positions
        this.yaw = [] //orientations [rad]
        this.directions = [] //directions (1:forward, -1:backward)
    }

    render() {
        if (this.x == []) {
            return;
        }
        ctx.beginPath();
        ctx.strokeStyle = 'green';
        ctx.moveTo(this.x[0], this.y[0]);
        for (let i = 1; i < this.x.length; i++) {
            ctx.lineTo(this.x[i], this.y[i]);
        }
        ctx.stroke();
    }
}