function canvas_arrow(context, fromx, fromy, tox, toy, color = 'red') {
    let headlen = 10;
    let arrow_len = 50;
    let dx = tox - fromx;
    let dy = toy - fromy;
    let angle = Math.atan2(dy, dx);
    let dist = Math.hypot(dx, dy);
    arrow_len = Math.min(dist, arrow_len);
    let tx = arrow_len * Math.cos(angle) + fromx;
    let ty = arrow_len * Math.sin(angle) + fromy;
    context.beginPath(); 
    context.lineCap = 'round';
    context.strokeStyle = color;
    context.moveTo(fromx, fromy);
    context.lineTo(tx, ty);
    context.lineTo(tx - headlen * Math.cos(angle - Math.PI / 6), ty - headlen * Math.sin(angle - Math.PI / 6));
    context.moveTo(tx, ty);
    context.lineTo(tx - headlen * Math.cos(angle + Math.PI / 6), ty - headlen * Math.sin(angle + Math.PI / 6));
    context.stroke();
}

function canvas_cross(context, tx, ty, color = 'red', size = 10) {
    context.beginPath();
    context.strokeStyle = color;
    context.moveTo(tx - size / 2.0 * Math.cos(Math.PI / 4), ty - size / 2.0 * Math.sin(Math.PI / 4));
    context.lineTo(tx + size / 2.0 * Math.cos(Math.PI / 4), ty + size / 2.0 * Math.sin(Math.PI / 4));
    context.moveTo(tx - size / 2.0 * Math.cos(Math.PI / 4), ty + size / 2.0 * Math.sin(Math.PI / 4));
    context.lineTo(tx + size / 2.0 * Math.cos(Math.PI / 4), ty - size / 2.0 * Math.sin(Math.PI / 4));
    context.stroke();
}