// Patch ros3djs LaserScan to filter out Infinity values
// This prevents the "white cloud" effect from max_range readings
(function patchLaserScanFilterInfinity() {
    const proto = ROS3D?.LaserScan?.prototype;
    if (!proto || proto.__infFilterPatched) return;

    const originalProcessMessage = proto.processMessage;
    if (!originalProcessMessage) {
        console.warn('[ros3d_patch] LaserScan.processMessage not found, skipping patch');
        return;
    }

    proto.processMessage = function (message) {
        // Filter out Infinity and values >= range_max
        if (message.ranges) {
            const maxRange = message.range_max || 20.0;
            message.ranges = message.ranges.map(r =>
                (!isFinite(r) || r >= maxRange) ? 0 : r
            );
        }
        return originalProcessMessage.call(this, message);
    };

    proto.__infFilterPatched = true;
    console.log('[ros3d_patch] LaserScan infinity filter applied');
})();

// Patch ros3djs OccupancyGridClient so it truly replaces the grid (no trails)
// Must run BEFORE you create any new ROS3D.OccupancyGridClient instances.
(function patchOccupancyGridClientNoTrails() {
            const proto = ROS3D?.OccupancyGridClient?.prototype;
            if (!proto || proto.__noTrailsPatched) return;

            proto.processMessage = function (message) {
                // Remove previous visualization (works across ros3djs variants)
                if (this.currentGrid) {
                    // If currentGrid is a SceneNode in your build, it may have its own TF subscription
                    if (this.currentGrid.unsubscribeTf) {
                        try {this.currentGrid.unsubscribeTf();} catch (e) { }
                    }

                    // Remove from both (safe even if it's not a child)
                    if (this.sceneNode && this.sceneNode.remove) this.sceneNode.remove(this.currentGrid);
                    if (this.rootObject && this.rootObject.remove) this.rootObject.remove(this.currentGrid);

                    if (this.currentGrid.dispose) this.currentGrid.dispose();
                    this.currentGrid = null;
                }

                const newGrid = new ROS3D.OccupancyGrid({
                    message,
                    color: this.color,
                    opacity: this.opacity
                });

                if (this.tfClient) {
                    // Keep ONE SceneNode alive; don't unsubscribe TF every update
                    if (!this.sceneNode || this.sceneNode.frameID !== message.header.frame_id) {
                        if (this.sceneNode) {
                            this.sceneNode.unsubscribeTf?.();
                            this.rootObject.remove(this.sceneNode);
                        }
                        this.sceneNode = new ROS3D.SceneNode({
                            frameID: message.header.frame_id,
                            tfClient: this.tfClient,
                            object: newGrid,
                            pose: this.offsetPose
                        });
                        this.rootObject.add(this.sceneNode);
                    } else {
                        this.sceneNode.add(newGrid);
                    }
                    this.currentGrid = newGrid;
                } else {
                    this.currentGrid = newGrid;
                    this.rootObject.add(newGrid);
                }

                this.emit?.("change");
            };

            proto.__noTrailsPatched = true;
        })();
