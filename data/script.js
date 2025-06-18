// Global variables
let currentSettings = {};
let updateInterval = null;

// DOM elements
document.addEventListener('DOMContentLoaded', function() {
    // Initialize the page
    initPage();
    
    // Set up event listeners
    setupEventListeners();
    
    // Start the periodic update
    startPeriodicUpdate();
});

function initPage() {
    // Initial data load
    fetchStatus();
}

function setupEventListeners() {
    // Charging enable/disable toggle
    document.getElementById('chargingEnabled').addEventListener('change', function() {
        const enabled = this.checked;
        showConfirmation(
            `${enabled ? 'Enable' : 'Disable'} Charging`,
            `Are you sure you want to ${enabled ? 'enable' : 'disable'} charging?`,
            function() {
                updateSetting({ chargingEnabled: enabled });
            }
        );
    });
    
    // Max current limit save button
    document.getElementById('saveCurrentLimit').addEventListener('click', function() {
        const maxCurrentLimit = parseFloat(document.getElementById('maxCurrentLimit').value);
        if (isNaN(maxCurrentLimit) || maxCurrentLimit <= 0) {
            alert('Please enter a valid current value');
            return;
        }
        updateSetting({ maxCurrentLimit: maxCurrentLimit });
    });
    
    // BMS mode radio buttons
    document.querySelectorAll('input[name="bmsMode"]').forEach(function(radio) {
        radio.addEventListener('change', function() {
            const mode = parseInt(this.value);
            // Show/hide manual settings div based on BMS mode
            if (mode === 0) { // MANUAL mode
                document.getElementById('manual-settings').style.display = 'block';
            } else {
                document.getElementById('manual-settings').style.display = 'none';
            }
        });
    });
    
    // Save manual settings button
    document.getElementById('saveManualSettings').addEventListener('click', function() {
        const targetVoltage = parseFloat(document.getElementById('targetVoltage').value);
        const targetCurrent = parseFloat(document.getElementById('targetCurrentInput').value);
        const cellCount = parseInt(document.getElementById('cellCount').value);
        const cellTargetVoltage = parseFloat(document.getElementById('cellTargetVoltage').value);
        const flatpackConfig = document.querySelector('input[name="flatpackConfig"]:checked').value;
        const useThreePhase = document.querySelector('input[name="powerMode"]:checked').value === 'true';
        
        if (isNaN(targetVoltage) || isNaN(targetCurrent) || 
            isNaN(cellCount) || isNaN(cellTargetVoltage)) {
            alert('Please enter valid values for all fields');
            return;
        }
        
        showConfirmation(
            'Save Settings',
            'Are you sure you want to save these settings?',
            function() {
                updateSetting({
                    targetVoltage: targetVoltage,
                    targetCurrent: targetCurrent,
                    cellCount: cellCount,
                    cellTargetVoltage: cellTargetVoltage,
                    flatpackConfig: parseInt(flatpackConfig),
                    useThreePhase: useThreePhase
                });
            }
        );
    });
    
    // Modal close button
    document.querySelector('.close-button').addEventListener('click', function() {
        closeModal();
    });
    
    // Modal cancel button
    document.getElementById('modal-cancel').addEventListener('click', function() {
        closeModal();
    });
}

function startPeriodicUpdate() {
    // Update status every 2 seconds
    updateInterval = setInterval(fetchStatus, 2000);
}

function fetchStatus() {
    fetch('/api/status')
        .then(response => response.json())
        .then(data => {
            updateUI(data);
        })
        .catch(error => {
            console.error('Error fetching status:', error);
            document.getElementById('systemStatus').textContent = 'Error connecting to device';
        });
}

function updateUI(data) {
    // Save current settings
    currentSettings = data;
    
    // Update charging status
    const chargingEnabledEl = document.getElementById('chargingEnabled');
    const chargingStatusEl = document.getElementById('chargingStatus');
    
    chargingEnabledEl.checked = data.chargingEnabled;
    chargingStatusEl.textContent = data.chargingEnabled ? 'Charging Enabled' : 'Charging Disabled';
    
    // Update max current limit
    document.getElementById('maxCurrentLimit').value = data.maxCurrentLimit;
    
    // Update battery data
    if (data.battery) {
        document.getElementById('batteryVoltage').textContent = data.battery.voltage.toFixed(2) + ' V';
        document.getElementById('batteryTemp').textContent = data.battery.temperatureMax.toFixed(1) + ' °C';
        document.getElementById('targetCurrent').textContent = data.battery.currentIn.toFixed(2) + ' A';
    }
    
    // Update Type2 status
    if (data.type2) {
        const stateNames = ['A: Not Connected', 'B: Connected/Not Ready', 'C: Charging', 'D: Charging with Ventilation', 'E: Error'];
        document.getElementById('type2State').textContent = stateNames[data.type2.state] || 'Unknown';
        document.getElementById('type2CurrentLimit').textContent = data.type2.allowedCurrent.toFixed(2) + ' A';
    }
    
    // Update BMS mode selection
    const bmsModeRadios = document.querySelectorAll('input[name="bmsMode"]');
    bmsModeRadios.forEach(radio => {
        radio.checked = parseInt(radio.value) === data.bmsMode;
    });
    
    // Update manual settings display
    document.getElementById('manual-settings').style.display = data.bmsMode === 0 ? 'block' : 'none';
    
    // Update manual settings values
    document.getElementById('targetVoltage').value = data.targetVoltage;
    document.getElementById('targetCurrentInput').value = data.targetCurrent;
    document.getElementById('cellCount').value = data.cellCount;
    document.getElementById('cellTargetVoltage').value = data.cellTargetVoltage;
    
    // Update Flatpack configuration
    const flatpackConfigRadios = document.querySelectorAll('input[name="flatpackConfig"]');
    flatpackConfigRadios.forEach(radio => {
        radio.checked = parseInt(radio.value) === data.flatpackConfig;
    });
    
    // Update power mode
    const powerModeRadios = document.querySelectorAll('input[name="powerMode"]');
    powerModeRadios.forEach(radio => {
        radio.checked = (radio.value === 'true') === data.useThreePhase;
    });
    
    // Update Flatpack status
    updateFlatpackStatus(data.flatpacks);
}

function updateFlatpackStatus(flatpacks) {
    const container = document.getElementById('flatpackContainer');
    container.innerHTML = ''; // Clear existing content
    
    if (!flatpacks || flatpacks.length === 0) {
        container.innerHTML = '<p>No Flatpacks connected</p>';
        return;
    }
    
    flatpacks.forEach(fp => {
        const fpCard = document.createElement('div');
        fpCard.className = 'flatpack-card';
        
        const status = fp.connected ? '<span class="connected">Connected</span>' : '<span class="disconnected">Disconnected</span>';
        
        fpCard.innerHTML = `
            <h3>Flatpack #${fp.id} - ${status}</h3>
            <div class="data-item">
                <span>Output:</span>
                <span>${fp.voltage.toFixed(2)} V / ${fp.current.toFixed(2)} A</span>
            </div>
            <div class="data-item">
                <span>Temperature:</span>
                <span>${fp.temperature1}°C / ${fp.temperature2}°C</span>
            </div>
            <div class="data-item">
                <span>Status:</span>
                <span>${getStatusText(fp)}</span>
            </div>
        `;
        
        container.appendChild(fpCard);
    });
}

function getStatusText(flatpack) {
    if (!flatpack.connected) return 'Disconnected';
    if (!flatpack.loggedIn) return 'Not Logged In';
    if (flatpack.isError) return 'Error';
    if (flatpack.isWalkingIn) return 'Walking In';
    if (flatpack.isCurrentLimiting) return 'Current Limiting';
    return 'Normal';
}

function updateSetting(settings) {
    fetch('/api/settings', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(settings)
    })
    .then(response => {
        if (!response.ok) {
            throw new Error('Failed to update settings');
        }
        return response.text();
    })
    .then(data => {
        document.getElementById('systemStatus').textContent = 'Settings updated successfully';
        setTimeout(() => {
            document.getElementById('systemStatus').textContent = '';
        }, 3000);
        
        // Force immediate status update
        fetchStatus();
    })
    .catch(error => {
        console.error('Error updating settings:', error);
        document.getElementById('systemStatus').textContent = 'Error updating settings';
    });
}

function showConfirmation(title, message, onConfirm) {
    document.getElementById('modal-title').textContent = title;
    document.getElementById('modal-message').textContent = message;
    
    // Set up confirm button action
    const confirmBtn = document.getElementById('modal-confirm');
    
    // Remove previous event listener using cloneNode
    const newConfirmBtn = confirmBtn.cloneNode(true);
    confirmBtn.parentNode.replaceChild(newConfirmBtn, confirmBtn);
    
    newConfirmBtn.addEventListener('click', function() {
        onConfirm();
        closeModal();
    });
    
    // Show the modal
    document.getElementById('modal').style.display = 'block';
}

function closeModal() {
    document.getElementById('modal').style.display = 'none';
}
