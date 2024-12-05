// containers/visualizer/app/static/script.js
let ws = new WebSocket('ws://' + window.location.host + '/ws');

// Initialize plots
let jointData = {
    type: 'scatter',
    mode: 'lines',
    x: [],
    y: []
};

let transformData = {
    type: 'scatter3d',
    mode: 'markers',
    x: [],
    y: [],
    z: []
};

Plotly.newPlot('jointPlot', [jointData], {
    title: 'Joint States',
    xaxis: {title: 'Time'},
    yaxis: {title: 'Position'}
});

Plotly.newPlot('transformPlot', [transformData], {
    title: 'Transforms'
});

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    
    if (data.type === 'joint_states') {
        const time = new Date().getTime();
        
        Plotly.extendTraces('jointPlot', {
            x: [[time]],
            y: [data.data.positions]
        }, [0]);
    }
    
    if (data.type === 'transforms') {
        const positions = data.data.map(t => t.translation);
        
        Plotly.update('transformPlot', {
            x: [positions.map(p => p.x)],
            y: [positions.map(p => p.y)],
            z: [positions.map(p => p.z)]
        }, [0]);
    }
};