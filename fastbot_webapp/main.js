// =============================================================================
// Environment Configuration
// Detects local vs Construct environment and sets URLs accordingly
// =============================================================================
const ENV_CONFIG = (function() {
    const isLocal = window.location.hostname === 'localhost' ||
                    window.location.hostname === '127.0.0.1';

    if (isLocal) {
        // Local development (Docker)
        return {
            rosbridge_url: 'ws://localhost:9090',
            video_server_host: 'localhost:11315',
            use_ssl: false,
        };
    } else {
        // The Construct - derive from current URL
        // Format: https://xxx.robotigniteacademy.com/uuid/...
        const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
        const pathParts = window.location.pathname.split('/').filter(p => p);
        const uuid = pathParts[0] || '';
        return {
            rosbridge_url: protocol + window.location.host + '/' + uuid + '/rosbridge/',
            video_server_host: window.location.host + '/' + uuid + '/cameras',
            use_ssl: window.location.protocol === 'https:',
        };
    }
})();

console.log('Environment config:', ENV_CONFIG);

let vueApp = new Vue({
    el: "#vueApp",
    data: {
        // ROS connection
        ros: null,
        rosbridge_address: ENV_CONFIG.rosbridge_url,
        connected: false,
        // Page content
        menu_title: 'Connection',
        // Dragging data
        dragging: false,
        x: 'no',
        y: 'no',
        dragCircleStyle: {
            margin: '0px',
            top: '0px',
            left: '0px',
            display: 'none',
            width: '75px',
            height: '75px',
        },
        // Joystick values
        joystick: {
            vertical: 0,
            horizontal: 0,
        },
        // Publisher interval
        pubInterval: null,
        // Map visualization
        mapViewer: null,
        mapGridClient: null,

        // Robot position tracking
        robotPosition: { x: null, y: null, theta: null },
        robotMarker: null,         // Container for footprint + arrow
        robotFootprint: null,      // PolygonMarker for footprint
        robotArrow: null,          // NavigationArrow for heading
        // Path visualization
        pathShape: null,
        pathTopic: null,
        // Pose topic subscriber
        poseTopic: null,
        // Costmap visualization
        globalCostmapClient: null,
        localCostmapClient: null,
        showGlobalCostmap: true,
        showLocalCostmap: true,
        // 3D stuff
        viewer: null,
        tfClient: null,
        urdfClient: null,
        laserScan3D: null,
        gridClient3D: null,
        globalCostmap3D: null,
        localCostmap3D: null, 
        path3D: null, 
        // Current cmd_vel display (commanded velocity)
        cmdVel: { linear: 0, angular: 0 },
        cmdVelTopic: null,
        // Waypoints
        waypoints: [
        ],
        selectedWaypointIndex: null,
    },
    methods: {
        connect: function() {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address,
                groovyCompatibility: false
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')
                
                this.pubInterval = setInterval(this.publish, 100)
                // Wait for Vue to render the DOM elements before setting up viewers
                this.$nextTick(() => {
                    this.setCamera()
                    this.setMap()
                    this.setup3DViewer()
                })
            })
            this.ros.on('error', (error) => {
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
                clearInterval(this.pubInterval)
                this.cleanupVisualization()
                document.getElementById('divCamera').innerHTML = ''
                document.getElementById('map').innerHTML = ''
                document.getElementById('div3DViewer').innerHTML = ''
            })
        },
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -this.joystick.horizontal, },
            })
            topic.publish(message)
        },
        disconnect: function() {
            this.ros.close()
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        setCamera: function() {
            // Use environment config for video server host
            const host = ENV_CONFIG.video_server_host;
            console.log('Video server host:', host);

            // Get container dimensions
            const container = document.getElementById('divCamera')
            const width = container.clientWidth || 800
            const height = container.clientHeight || 500

            // Primary camera view - scales to container
            new MJPEGCANVAS.Viewer({
                divID: 'divCamera',
                host: host,
                width: width,
                height: height,
                topic: '/fastbot_camera/image_raw',
                ssl: ENV_CONFIG.use_ssl,
            })
        },

        setup3DViewer() {
            // 3D viewer (right side panel - 30% width)
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: 'div3DViewer',
                width: 350,
                height: 220,
                antialias: true
            })

            // Position camera to see robot (small robot ~0.1-0.2m)
            // this.viewer.camera.position.set(0.3, 0.3, 0.3)
            // this.viewer.camera.lookAt(0, 0, 0)
            // console.log('[3D] Camera position set to:', this.viewer.camera.position)

            const zPose = (z) => new ROSLIB.Pose({
                position: { x: 0, y: 0, z },
                orientation: { x: 0, y: 0, z: 0, w: 1 }
            });

            // Add a grid.
            this.viewer.addObject(new ROS3D.Grid({
                color:'#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))

            // Setup a client to listen to TFs.
            // ROSLIB.TFClient has ROS1/ROS2 action incompatibility, so we use topic-based mode
            // by NOT providing serverName, which forces direct /tf subscription
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 15.0,
                fixedFrame: 'map',
                // Omit serverName to avoid action client and use direct topic subscription
            })


            // Debug: Test if TFClient can resolve a transform
            // setTimeout(() => {
            //     console.log('[TF] Testing TFClient transform lookup for fastbot_base_link...')
            //     console.log('[TF] TFClient internal state:', {
            //         serverName: this.tfClient.serverName,
            //         fixedFrame: this.tfClient.fixedFrame,
            //         frameInfos: Object.keys(this.tfClient.frameInfos || {})
            //     })
            //     console.log(this.tfClient)
            //     this.tfClient.subscribe('fastbot_base_link', (transform) => {
            //         console.log('[TF] ✓ Got transform for fastbot_base_link:', transform)
            //     })
            // }, 2000)

            // Add map frame axes (X=red, Y=green, Z=blue)
            this.mapAxes = new ROS3D.Axes({
                ros: this.ros,
                tfClient: this.tfClient,
                frame_id: 'map',
                shaftRadius: 0.02,
                headRadius: 0.05,
                headLength: 0.1,
                scale: 1.0
            })
            this.viewer.scene.add(this.mapAxes)

            // Setup the URDF client.
            // Note: ros3djs converts package://fastbot_description/... to {path}/fastbot_description/...
            // Make sure the path ends without trailing slash since package:// paths start with the package name
            const basePath = (location.origin + location.pathname).replace(/\/+$/, '')
            console.log('[3D] URDF base path:', basePath)
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: '/fastbot_robot_state_publisher:robot_description',
                tfClient: this.tfClient,
                path: basePath,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })

            // Add the occupancy grid (map) in 3D at z=0
            this.gridClient3D = new ROS3D.OccupancyGridClient({
                ros: this.ros,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                topic: '/map',
                continuous: false,
                opacity: 1.0,
                offsetPose: zPose(0.00),
            })

            // Add Global Costmap in 3D (z=0.01 to avoid z-fighting)
            this.globalCostmap3D = new ROS3D.OccupancyGridClient({
                ros: this.ros,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                topic: '/global_costmap/costmap',
                continuous: false,
                opacity: 0.4,
                offsetPose: zPose(0.01),
            })

            // Add Local Costmap in 3D (z=0.02)
            this.localCostmap3D = new ROS3D.OccupancyGridClient({
                ros: this.ros,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                topic: '/local_costmap/costmap',
                continuous: true,
                opacity: 0.5,
                offsetPose: zPose(0.02),
            })

            // Add Laser Scan in 3D (disabled - causes performance issues)
            // this.laserScan3D = new ROS3D.LaserScan({
            //     ros: this.ros,
            //     tfClient: this.tfClient,
            //     rootObject: this.viewer.scene,
            //     topic: '/fastbot/scan',
            //     material: { color: 0xff0000, size: 0.02 },
            //     max_pts: 300,
            // })

            // Add Path in 3D
            this.path3D = new ROS3D.Path({
                ros: this.ros,
                tfClient: this.tfClient,
                rootObject: this.viewer.scene,
                topic: '/plan',
            })

        },

        setMap: function() {
            // 2D Map (right side panel)
            this.mapViewer = new ROS2D.Viewer({
                divID: 'map',
                width: 350,
                height: 250
            })

            // Setup the map client
            this.mapGridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.mapViewer.scene,
                continuous: true,
            })

            // Scale the canvas to fit to the map
            this.mapGridClient.on('change', () => {
                this.mapViewer.scaleToDimensions(
                    this.mapGridClient.currentGrid.width,
                    this.mapGridClient.currentGrid.height
                )
                this.mapViewer.shift(
                    this.mapGridClient.currentGrid.pose.position.x,
                    this.mapGridClient.currentGrid.pose.position.y
                )
                // Setup costmaps, robot marker and path after map is loaded
                this.setupCostmaps()
                this.setupRobotMarker()
                this.setupPathVisualization()
            })
        },

        setupCostmaps: function() {
            // Global costmap (covers entire map, shows inflated obstacles)
            if (this.showGlobalCostmap) {
                this.globalCostmapClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: this.mapViewer.scene,
                    topic: '/global_costmap/costmap',
                    continuous: true,
                })
                // Set opacity when costmap updates
                this.globalCostmapClient.on('change', () => {
                    if (this.globalCostmapClient.currentGrid) {
                        this.globalCostmapClient.currentGrid.alpha = 0.5
                    }
                })
            }

            // Local costmap (smaller area around robot, for local planning)
            if (this.showLocalCostmap) {
                this.localCostmapClient = new ROS2D.OccupancyGridClient({
                    ros: this.ros,
                    rootObject: this.mapViewer.scene,
                    topic: '/local_costmap/costmap',
                    continuous: true,
                })
                // Set opacity when costmap updates
                this.localCostmapClient.on('change', () => {
                    if (this.localCostmapClient.currentGrid) {
                        this.localCostmapClient.currentGrid.alpha = 0.4
                    }
                })
            }
        },

        setupRobotMarker: function() {
            // Robot footprint from Nav2 config (in meters)
            // footprint: "[ [0.089, 0.069], [0.089, -0.069], [-0.089, -0.069], [-0.089, 0.069] ]"
            const footprint = [
                { x: 0.089, y: 0.069 },
                { x: 0.089, y: -0.069 },
                { x: -0.089, y: -0.069 },
                { x: -0.089, y: 0.069 },
                { x: 0.089, y: 0.069 }
            ]

            // Create a container to hold both footprint and arrow
            this.robotMarker = new createjs.Container()

            // Create footprint using ROS2D.PolygonMarker
            this.robotFootprint = new ROS2D.PolygonMarker({
                lineSize: 0.01,
                lineColor: createjs.Graphics.getRGB(0, 0, 0, 1),
                pointSize: 0.01,
                pointColor: createjs.Graphics.getRGB(0, 255, 0, 0.8),
                fillColor: createjs.Graphics.getRGB(50, 255, 0, 0.5)
            })

            // Add footprint points to polygon
            footprint.forEach(point => {
                this.robotFootprint.addPoint(new ROSLIB.Vector3({ x: point.x, y: point.y }))
            })
            this.robotMarker.addChild(this.robotFootprint)

            // Create navigation arrow to show heading direction
            this.robotArrow = new ROS2D.ArrowShape({
                size: 0.15,
                strokeSize: 0.01,
                strokeColor: createjs.Graphics.getRGB(0, 0, 0),
                fillColor: createjs.Graphics.getRGB(0, 0, 200),
                pulse: true
            })
            this.robotMarker.addChild(this.robotArrow)

            this.robotMarker.visible = false
            this.mapViewer.scene.addChild(this.robotMarker)

            // Subscribe to robot pose topic (AMCL pose or odom)
            this.poseTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/amcl_pose',
                messageType: 'geometry_msgs/PoseWithCovarianceStamped'
            })

            this.poseTopic.subscribe((message) => {
                this.updateRobotPosition(message.pose.pose)
            })

            // Subscribe to cmd_vel for speed display (shows commands from any source)
            this.cmdVelTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/fastbot/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })

            this.cmdVelTopic.subscribe((message) => {
                this.cmdVel.linear = message.linear.x
                this.cmdVel.angular = message.angular.z
            })
        },

        updateRobotPosition: function(pose) {
            // Update robot position data
            this.robotPosition.x = pose.position.x
            this.robotPosition.y = pose.position.y

            // Calculate orientation from quaternion
            const q = pose.orientation
            this.robotPosition.theta = Math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )

            // Update robot marker position and rotation (2D map)
            if (this.robotMarker) {
                this.robotMarker.x = pose.position.x
                this.robotMarker.y = -pose.position.y
                this.robotMarker.rotation = -this.robotPosition.theta * (180 / Math.PI)
                this.robotMarker.visible = true
            }
        },

        setupPathVisualization: function() {
            // Create path shape for visualization
            this.pathShape = new ROS2D.PathShape({
                strokeSize: 0.02,
                strokeColor: createjs.Graphics.getRGB(0, 128, 255)
            })
            this.mapViewer.scene.addChild(this.pathShape)

            // Subscribe to path topic (from navigation planner)
            this.pathTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: 'plan',
                messageType: 'nav_msgs/Path'
            })

            this.pathTopic.subscribe((message) => {
                if (message.poses && message.poses.length > 0) {
                    this.pathShape.setPath(message)
                }
            })
        },

        cleanupVisualization: function() {
            // Unsubscribe from topics
            if (this.poseTopic) {
                this.poseTopic.unsubscribe()
                this.poseTopic = null
            }
            if (this.pathTopic) {
                this.pathTopic.unsubscribe()
                this.pathTopic = null
            }
            if (this.cmdVelTopic) {
                this.cmdVelTopic.unsubscribe()
                this.cmdVelTopic = null
            }
            // Reset costmap clients
            this.globalCostmapClient = null
            this.localCostmapClient = null
            // Reset markers
            this.robotMarker = null
            this.robotFootprint = null
            this.robotArrow = null
            this.pathShape = null
            this.robotPosition = { x: null, y: null, theta: null }
            this.cmdVel = { linear: 0, angular: 0 }
        },

        goToSelectedWaypoint: function() {
            if (this.selectedWaypointIndex === null) return
            const waypoint = this.waypoints[this.selectedWaypointIndex]

            let goalTopic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/goal_pose',
                messageType: 'geometry_msgs/msg/PoseStamped'
            });

            const thetaDeg = waypoint.theta * 180 / Math.PI;
            const qz = Math.sin(waypoint.theta / 2.0);
            const qw = Math.cos(waypoint.theta / 2.0);

            let message = new ROSLIB.Message({
                header: {
                    frame_id: 'fastbot_odom',
                    stamp: { sec: 0, nanosec: 0 }
                },
                pose: {
                    position: { x: waypoint.x, y: waypoint.y, z: 0.0 },
                    orientation: { x: 0.0, y: 0.0, z: qz, w: qw }
                }
            });

            goalTopic.publish(message);
            console.log(`Navigate to waypoint: ${waypoint.name} (${waypoint.x}, ${waypoint.y.toFixed(2)}, theta=${thetaDeg}°)`)
        },

    },
    mounted() {
        // Page is ready
        window.addEventListener('mouseup', this.stopDrag)
    },
})