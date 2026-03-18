// particles-bg.js - 纯 Canvas 实现（带鼠标交互）
(function() {
    const canvas = document.getElementById('background');
    if (!canvas) {
        console.error('Background canvas not found');
        return;
    }
    
    const ctx = canvas.getContext('2d');
    let particles = [];
    const particleCount = 80;
    const connectionDistance = 150;
    const mouseInteractionDistance = 200; // 鼠标影响范围
    
    // 鼠标位置
    let mouse = {
        x: null,
        y: null,
        radius: mouseInteractionDistance
    };
    
    // 鼠标拖尾轨迹
    let mouseTrail = [];
    const maxTrailLength = 10;
    
    // 设置 canvas 尺寸
    function resize() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }
    resize();
    window.addEventListener('resize', resize);
    
    // 跟踪鼠标位置
    document.addEventListener('mousemove', (e) => {
        mouse.x = e.clientX;
        mouse.y = e.clientY;
        
        // 添加鼠标轨迹点
        mouseTrail.push({ x: e.clientX, y: e.clientY, time: Date.now() });
        if (mouseTrail.length > maxTrailLength) {
            mouseTrail.shift();
        }
    });
    
    // 鼠标离开时清除位置
    document.addEventListener('mouseleave', () => {
        mouse.x = null;
        mouse.y = null;
    });
    
    // 创建粒子
    class Particle {
        constructor() {
            this.x = Math.random() * canvas.width;
            this.y = Math.random() * canvas.height;
            this.vx = (Math.random() - 0.5) * 2;
            this.vy = (Math.random() - 0.5) * 2;
            this.radius = 3;
            this.baseRadius = 3;
            
            // 随机运动参数
            this.randomChangeTimer = 0; // 随机方向变化计时器
            this.randomChangeInterval = Math.random() * 2000 + 1000; // 1-3秒随机变化一次
            this.targetVx = this.vx; // 目标速度
            this.targetVy = this.vy;
            this.maxSpeed = 2 + Math.random() * 1; // 每个粒子的最大速度不同
        }
        
        update() {
            // 随机方向变化：每隔一段时间改变运动方向
            this.randomChangeTimer++;
            if (this.randomChangeTimer >= this.randomChangeInterval) {
                // 生成新的随机目标速度
                const angle = Math.random() * Math.PI * 2;
                const speed = 0.5 + Math.random() * this.maxSpeed;
                this.targetVx = Math.cos(angle) * speed;
                this.targetVy = Math.sin(angle) * speed;
                
                // 重置计时器，设置下一次变化的间隔
                this.randomChangeTimer = 0;
                this.randomChangeInterval = Math.random() * 2000 + 1000;
            }
            
            // 平滑过渡到目标速度（让运动更自然）
            this.vx += (this.targetVx - this.vx) * 0.05;
            this.vy += (this.targetVy - this.vy) * 0.05;
            
            // 添加随机扰动（布朗运动效果）
            this.vx += (Math.random() - 0.5) * 0.1;
            this.vy += (Math.random() - 0.5) * 0.1;
            
            // 限制最大速度
            const speed = Math.sqrt(this.vx * this.vx + this.vy * this.vy);
            if (speed > this.maxSpeed) {
                this.vx = (this.vx / speed) * this.maxSpeed;
                this.vy = (this.vy / speed) * this.maxSpeed;
            }
            
            // 鼠标交互：推开附近的粒子
            if (mouse.x !== null && mouse.y !== null) {
                const dx = mouse.x - this.x;
                const dy = mouse.y - this.y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                
                if (distance < mouse.radius) {
                    // 计算推开力（距离越近，力越大）
                    const force = (mouse.radius - distance) / mouse.radius;
                    const angle = Math.atan2(dy, dx);
                    
                    // 粒子被推开
                    this.vx -= Math.cos(angle) * force * 0.5;
                    this.vy -= Math.sin(angle) * force * 0.5;
                    
                    // 鼠标附近粒子变大
                    this.radius = this.baseRadius + force * 2;
                } else {
                    // 恢复原始大小
                    this.radius += (this.baseRadius - this.radius) * 0.1;
                }
            } else {
                // 恢复原始大小
                this.radius += (this.baseRadius - this.radius) * 0.1;
            }
            
            // 更新位置
            this.x += this.vx;
            this.y += this.vy;
            
            // 边界处理：反弹并添加随机扰动
            if (this.x < 0) {
                this.x = 0;
                this.vx = Math.abs(this.vx) + Math.random() * 0.5;
                // 改变目标速度
                this.targetVx = Math.random() * this.maxSpeed;
            }
            if (this.x > canvas.width) {
                this.x = canvas.width;
                this.vx = -Math.abs(this.vx) - Math.random() * 0.5;
                this.targetVx = -Math.random() * this.maxSpeed;
            }
            if (this.y < 0) {
                this.y = 0;
                this.vy = Math.abs(this.vy) + Math.random() * 0.5;
                this.targetVy = Math.random() * this.maxSpeed;
            }
            if (this.y > canvas.height) {
                this.y = canvas.height;
                this.vy = -Math.abs(this.vy) - Math.random() * 0.5;
                this.targetVy = -Math.random() * this.maxSpeed;
            }
            
            // 限制在画布内（双重保险）
            this.x = Math.max(0, Math.min(canvas.width, this.x));
            this.y = Math.max(0, Math.min(canvas.height, this.y));
            
            // 轻微的速度衰减（保持运动活力）
            this.vx *= 0.99;
            this.vy *= 0.99;
        }
        
        draw() {
            ctx.beginPath();
            ctx.arc(this.x, this.y, this.radius, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(135, 206, 250, 0.6)';
            ctx.fill();
        }
    }
    
    // 初始化粒子
    for (let i = 0; i < particleCount; i++) {
        particles.push(new Particle());
    }
    
    // 绘制粒子之间的连线
    function drawParticleLines() {
        for (let i = 0; i < particles.length; i++) {
            for (let j = i + 1; j < particles.length; j++) {
                const dx = particles[i].x - particles[j].x;
                const dy = particles[i].y - particles[j].y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                
                if (distance < connectionDistance) {
                    ctx.beginPath();
                    ctx.moveTo(particles[i].x, particles[i].y);
                    ctx.lineTo(particles[j].x, particles[j].y);
                    ctx.strokeStyle = `rgba(135, 206, 250, ${0.4 * (1 - distance / connectionDistance)})`;
                    ctx.lineWidth = 1;
                    ctx.stroke();
                }
            }
        }
    }
    
    // 绘制鼠标与粒子的连线
    function drawMouseLines() {
        if (mouse.x === null || mouse.y === null) return;
        
        particles.forEach(particle => {
            const dx = mouse.x - particle.x;
            const dy = mouse.y - particle.y;
            const distance = Math.sqrt(dx * dx + dy * dy);
            
            if (distance < mouseInteractionDistance) {
                ctx.beginPath();
                ctx.moveTo(mouse.x, mouse.y);
                ctx.lineTo(particle.x, particle.y);
                // 距离越近，连线越亮
                const opacity = 0.3 * (1 - distance / mouseInteractionDistance);
                ctx.strokeStyle = `rgba(255, 182, 193, ${opacity})`;
                ctx.lineWidth = 1;
                ctx.stroke();
            }
        });
    }
    
        // 绘制鼠标拖尾
        // 绘制鼠标拖尾（带渐变效果）
        // 绘制鼠标拖尾（整体渐变效果，从最新点到最旧点）
    function drawMouseTrail() {
        if (mouseTrail.length < 2) return;
        
        const now = Date.now();
        const fadeTime = 500; // 拖尾消失时间（毫秒）
        
        // 过滤掉过期的轨迹点
        const validTrail = mouseTrail.filter(point => (now - point.time) < fadeTime);
        if (validTrail.length < 2) return;
        
        // 找到最新点和最旧点（用于整体渐变）
        const newestPoint = validTrail[validTrail.length - 1];
        const oldestPoint = validTrail[0];
        const totalAge = newestPoint.time - oldestPoint.time;
        
        // 如果轨迹太短，使用分段渐变
        if (totalAge < 50) {
            // 分段绘制，每段都有渐变
            for (let i = 1; i < validTrail.length; i++) {
                const currentPoint = validTrail[i];
                const prevPoint = validTrail[i - 1];
                
                const currentAge = now - currentPoint.time;
                const prevAge = now - prevPoint.time;
                
                const currentOpacity = (1 - currentAge / fadeTime) * 0.6;
                const prevOpacity = (1 - prevAge / fadeTime) * 0.6;
                
                const gradient = ctx.createLinearGradient(
                    prevPoint.x, prevPoint.y,
                    currentPoint.x, currentPoint.y
                );
                
                gradient.addColorStop(0, `rgba(255, 182, 193, ${prevOpacity})`);
                gradient.addColorStop(1, `rgba(255, 182, 193, ${currentOpacity})`);
                
                ctx.beginPath();
                ctx.moveTo(prevPoint.x, prevPoint.y);
                ctx.lineTo(currentPoint.x, currentPoint.y);
                ctx.strokeStyle = gradient;
                ctx.lineWidth = 3;
                ctx.lineCap = 'round';
                ctx.stroke();
            }
        } else {
            // 整体渐变：从最新点（深）到最旧点（浅）
            const gradient = ctx.createLinearGradient(
                newestPoint.x, newestPoint.y,
                oldestPoint.x, oldestPoint.y
            );
            
            // 最新点：最深（不透明度最高）
            const newestOpacity = (1 - (now - newestPoint.time) / fadeTime) * 0.8;
            // 最旧点：最浅（不透明度最低）
            const oldestOpacity = (1 - (now - oldestPoint.time) / fadeTime) * 0.2;
            
            gradient.addColorStop(0, `rgba(255, 182, 193, ${newestOpacity})`);
            gradient.addColorStop(1, `rgba(255, 182, 193, ${oldestOpacity})`);
            
            // 绘制整个拖尾路径
            ctx.beginPath();
            ctx.moveTo(validTrail[0].x, validTrail[0].y);
            for (let i = 1; i < validTrail.length; i++) {
                ctx.lineTo(validTrail[i].x, validTrail[i].y);
            }
            ctx.strokeStyle = gradient;
            ctx.lineWidth = 3;
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';
            ctx.stroke();
        }
        
        // 清理过期的轨迹点
        mouseTrail = mouseTrail.filter(point => (now - point.time) < fadeTime);
    }
    
    // 绘制鼠标光标效果
    function drawMouseCursor() {
        if (mouse.x === null || mouse.y === null) return;
        
        // 绘制鼠标周围的渐变圆圈
        const gradient = ctx.createRadialGradient(mouse.x, mouse.y, 0, mouse.x, mouse.y, mouse.radius);
        gradient.addColorStop(0, 'rgba(255, 182, 193, 0.1)');
        gradient.addColorStop(1, 'rgba(255, 182, 193, 0)');
        
        ctx.beginPath();
        ctx.arc(mouse.x, mouse.y, mouse.radius, 0, Math.PI * 2);
        ctx.fillStyle = gradient;
        ctx.fill();
    }
    
    // 动画循环
    function animate() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // 更新粒子
        particles.forEach(particle => {
            particle.update();
        });
        
        // 绘制鼠标拖尾（最先绘制，作为背景）
        drawMouseTrail();
        
        // 绘制鼠标光标效果
        drawMouseCursor();
        
        // 绘制粒子之间的连线
        drawParticleLines();
        
        // 绘制鼠标与粒子的连线
        drawMouseLines();
        
        // 绘制粒子（最后绘制，确保在最上层）
        particles.forEach(particle => {
            particle.draw();
        });
        
        requestAnimationFrame(animate);
    }
    
    animate();
})();