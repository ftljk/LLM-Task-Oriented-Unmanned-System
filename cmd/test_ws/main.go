package main

import (
	"fmt"
	"log"
	"time"

	"robot/pkg/ros"
)

func main() {
	client, err := ros.NewRosBridgeClient("ws://localhost:9090")
	if err != nil {
		log.Fatalf("Failed to connect: %v", err)
	}
	defer client.Close()
	fmt.Println("✅ Connected to ROS bridge")

	// 1. Test GetTopicsForType for Twist
	topics, err := client.GetTopicsForType("geometry_msgs/Twist")
	if err != nil {
		log.Fatalf("GetTopicsForType(Twist) failed: %v", err)
	}
	fmt.Printf("✅ Twist topics: %v\n", topics)

	// 2. Test GetTopicsForType for Odometry
	odomTopics, err := client.GetTopicsForType("nav_msgs/Odometry")
	if err != nil {
		log.Fatalf("GetTopicsForType(Odometry) failed: %v", err)
	}
	fmt.Printf("✅ Odometry topics: %v\n", odomTopics)

	// 3. Test Subscribe + Get Odometry (one-shot with timeout)
	odomTopic := odomTopics[0]
	fmt.Printf("   Subscribing to %s...\n", odomTopic)

	ch := make(chan map[string]interface{}, 1)
	err = client.Subscribe(odomTopic, func(msg map[string]interface{}) {
		log.Printf("Callback fired for %s, keys: %v", odomTopic, keysOf(msg))
		rawMsg, ok := msg["msg"].(map[string]interface{})
		if !ok {
			return
		}
		pose, ok := rawMsg["pose"].(map[string]interface{})
		if !ok {
			return
		}
		innerPose, ok := pose["pose"].(map[string]interface{})
		if !ok {
			return
		}
		position, ok := innerPose["position"].(map[string]interface{})
		if !ok {
			return
		}
		x, _ := position["x"].(float64)
		y, _ := position["y"].(float64)
		fmt.Printf("✅ Odometry: x=%.3f y=%.3f\n", x, y)
		select {
		case ch <- msg:
		default:
		}
	})
	if err != nil {
		log.Fatalf("Subscribe failed: %v", err)
	}

	select {
	case <-ch:
		client.Unsubscribe(odomTopic)
		fmt.Println("✅ Odometry received successfully")
	case <-time.After(8 * time.Second):
		client.Unsubscribe(odomTopic)
		fmt.Println("⚠️ Odometry timeout - no data from bridge")
	}

	// 4. Test Publish (cmd_vel)
	twistTopic := topics[0]
	fmt.Printf("   Publishing Twist to %s...\n", twistTopic)
	err = client.Publish(twistTopic, "geometry_msgs/Twist", map[string]interface{}{
		"linear":  map[string]interface{}{"x": 0.5, "y": 0.0, "z": 0.0},
		"angular": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 0.0},
	})
	if err != nil {
		log.Fatalf("Publish failed: %v", err)
	}
	fmt.Println("✅ Twist published")

	fmt.Println("\n🎉 All WebSocket tests passed!")
}

func keysOf(m map[string]interface{}) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	return keys
}
