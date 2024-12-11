document.addEventListener('DOMContentLoaded', () => {
    const statusEl = document.getElementById('status');
    const dataBody = document.getElementById('dataBody');
    const logWrapper = document.getElementById('logWrapper');
    const autoScrollCheckbox = document.getElementById('autoScrollLog');
  
    // Toggles
    const toggles = {
      jointPos: document.getElementById('showJointPos'),
      jointVel: document.getElementById('showJointVel'),
      jointEff: document.getElementById('showJointEff'),
      tfPos:   document.getElementById('showTfPos'),
      tfRot:   document.getElementById('showTfRot')
    };
  
    let counter = 0;
    const maxPoints = 200;
  
    const jointPosData = {time:[], left:[], right:[]};
    const jointVelData = {time:[], left:[], right:[]};
    const jointEffData = {time:[], left:[], right:[]};
    const tfPosData   = {time:[], left:[], right:[]};
    const tfRotData   = {time:[], left:[], right:[]};
  
    const chartJointPos = createDualLineChart('chartJointPos','Position');
    const chartJointVel = createDualLineChart('chartJointVel','Velocity');
    const chartJointEff = createDualLineChart('chartJointEff','Effort');
    const chartTfPos = createDualLineChart('chartTfPos','X Pos');
    const chartTfRot = createDualLineChart('chartTfRot','Rot Z');
  
    Object.values(toggles).forEach(el=>el.addEventListener('change', updateVisibility));
    updateVisibility();
  
    function updateVisibility() {
      document.getElementById('cardJointPos').style.display = toggles.jointPos.checked?'block':'none';
      document.getElementById('cardJointVel').style.display = toggles.jointVel.checked?'block':'none';
      document.getElementById('cardJointEff').style.display = toggles.jointEff.checked?'block':'none';
      document.getElementById('cardTfPos').style.display = toggles.tfPos.checked?'block':'none';
      document.getElementById('cardTfRot').style.display = toggles.tfRot.checked?'block':'none';
    }
  
    // Throttle chart updates
    let lastUpdate = 0;
    const updateInterval = 500; // ms
  
    function maybeUpdateCharts() {
      const now = Date.now();
      if (now - lastUpdate > updateInterval) {
        updateCharts();
        lastUpdate = now;
      }
    }
  
    function connectWS() {
      const ws = new WebSocket(`ws://${window.location.host}/ws`);
      ws.onopen = () => {
        statusEl.textContent = 'Connected';
        statusEl.className = 'status status-connected';
      };
      ws.onclose = () => {
        statusEl.textContent = 'Disconnected - Retrying...';
        statusEl.className = 'status status-error';
        setTimeout(connectWS, 1000);
      };
      ws.onerror = () => {
        statusEl.textContent = 'Error';
        statusEl.className = 'status status-error';
      };
      ws.onmessage = evt => {
        const data = JSON.parse(evt.data);
        counter++;
  
        let dt;
        if (typeof data.timestamp === 'string') {
          dt = new Date(data.timestamp);
        } else {
          dt = new Date(data.timestamp*1000);
        }
        const timeStr = dt.toLocaleTimeString();
  
        let info = '';
        if (data.type === 'joint_states' || data.joint_names) {
          // Detailed joint data
          const names = data.joint_names || (data.data && data.data.names) || [];
          const positions = data.positions || (data.data && data.data.positions) || [];
          const velocities = data.velocities || (data.data && data.data.velocities) || [];
          const efforts = data.efforts || (data.data && data.data.efforts) || [];
  
          const ln = names.indexOf('left_wheel_joint');
          const rn = names.indexOf('right_wheel_joint');
  
          const lPos=(ln>=0?positions[ln]:0), rPos=(rn>=0?positions[rn]:0);
          const lVel=(ln>=0?velocities[ln]:0), rVel=(rn>=0?velocities[rn]:0);
          const lEff=(ln>=0?efforts[ln]:0), rEff=(rn>=0?efforts[rn]:0);
  
          // Show all details without heavy truncation
          info = `Left: pos=${lPos}, vel=${lVel}, eff=${lEff} | Right: pos=${rPos}, vel=${rVel}, eff=${rEff}`;
  
          pushData(jointPosData, dt, lPos, rPos);
          pushData(jointVelData, dt, lVel, rVel);
          pushData(jointEffData, dt, lEff, rEff);
  
        } else if (data.type === 'transforms' || data.transforms) {
          // Detailed transform data
          const transforms = data.transforms || data.data || [];
          const left = transforms.find(d=>d.child_frame_id==='left_wheel');
          const right= transforms.find(d=>d.child_frame_id==='right_wheel');
  
          const lX=left?left.translation.x:0;
          const rX=right?right.translation.x:0;
          const lZ=left?left.rotation.z:0;
          const rZ=right?right.rotation.z:0;
  
          // Show full transforms for clarity
          info = `Left wheel: x=${lX}, y=${left?left.translation.y:0}, z=${left?left.translation.z:0}, rot=(${left?left.rotation.x:0}, ${left?left.rotation.y:0}, ${lZ}, ${left?left.rotation.w:0})
                  | Right wheel: x=${rX}, y=${right?right.translation.y:0}, z=${right?right.translation.z:0}, rot=(${right?right.rotation.x:0}, ${right?right.rotation.y:0}, ${rZ}, ${right?right.rotation.w:0})`;
  
          pushData(tfPosData, dt, lX, rX);
          pushData(tfRotData, dt, lZ, rZ);
        }
  
        const row = document.createElement('tr');
        row.className = (data.type==='joint_states' || data.joint_names)? 'row-joint':'row-tf';
        row.innerHTML = `<td>${counter}</td><td>${timeStr}</td><td>${data.type||'N/A'}</td><td>${info}</td>`;
        dataBody.appendChild(row);
        if (dataBody.children.length > 1000) dataBody.removeChild(dataBody.children[0]);
  
        // Autoscroll
        if (autoScrollCheckbox.checked) {
          logWrapper.scrollTop = logWrapper.scrollHeight;
        }
  
        maybeUpdateCharts();
      };
    }
    connectWS();
  
    function pushData(store, time, leftVal, rightVal) {
      store.time.push(time);
      store.left.push(leftVal);
      store.right.push(rightVal);
      if (store.time.length > maxPoints) {
        store.time.shift();
        store.left.shift();
        store.right.shift();
      }
    }
  
    function updateCharts() {
      // Just set data. No manual min/max for Y. Let chart.js auto scale dynamically.
      setChartData(chartJointPos, jointPosData);
      setChartData(chartJointVel, jointVelData);
      setChartData(chartJointEff, jointEffData);
      setChartData(chartTfPos, tfPosData);
      setChartData(chartTfRot, tfRotData);
    }
  
    function setChartData(chart, store) {
      chart.data.labels = store.time.map(t=>t.toLocaleTimeString());
      chart.data.datasets[0].data = store.left;
      chart.data.datasets[1].data = store.right;
      chart.update();
    }
  
    function createDualLineChart(id, yLabel) {
      const ctx = document.getElementById(id).getContext('2d');
      return new Chart(ctx, {
        type:'line',
        data:{
          labels:[],
          datasets:[
            {
              label:'Left',
              data:[],
              // borderColor:'#1f77b4',
              borderColor: 'rgba(31, 119, 180, 0.8)', // #1f77b4 with alpha
              backgroundColor: 'rgba(31, 119, 180, 0.1)',
              fill:false,
              pointRadius:0,
              borderWidth:2
            },
            {
              label:'Right',
              data:[],
              // borderColor:'#2ca02c',
              borderColor: 'rgba(44, 160, 44, 0.8)', // #2ca02c with alpha
              backgroundColor: 'rgba(44, 160, 44, 0.1)',
              fill:false,
              pointRadius:0,
              borderWidth:2,
              borderDash:[5,5]
            }
          ]
        },
        options:{
          responsive:true,
          maintainAspectRatio:false,
          animation:false,
          plugins:{
            decimation:{
              enabled:true,
              algorithm:'lttb',
              samples:50
            },
            zoom:{
              zoom:{
                wheel:{ enabled:true },
                pinch:{ enabled:true },
                mode:'xy'
              },
              pan:{
                enabled:true,
                mode:'xy'
              }
            }
          },
          scales:{
            x:{
              title:{display:true,text:'Time'}
            },
            y:{
              title:{display:true,text:yLabel},
              // No manual min/max, let chart.js handle dynamic scaling
            }
          }
        }
      });
    }
  });