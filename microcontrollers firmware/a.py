import pygame
import random
import math
import os

# Инициализация Pygame
pygame.init()
pygame.mixer.init()

# Константы
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# Цвета
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
CYAN = (0, 255, 255)

# Настройка экрана
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Космическая Стрелялка")
clock = pygame.time.Clock()

# Шрифты
font_small = pygame.font.SysFont('Arial', 20)
font_medium = pygame.font.SysFont('Arial', 30)
font_large = pygame.font.SysFont('Arial', 40)

# Звуковые эффекты (создаем простые звуки)
def create_sound(frequency, duration):
    sample_rate = 22050
    n_samples = int(round(duration * sample_rate))
    buf = bytearray(n_samples)
    for i in range(n_samples):
        t = float(i) / sample_rate
        buf[i] = int(127 * math.sin(2 * math.pi * frequency * t))
    sound = pygame.sndarray.make_sound(pygame.array.array('u1', buf))
    return sound

# Создаем звуковые эффекты
shoot_sound = create_sound(440, 0.1)
explosion_sound = create_sound(220, 0.3)
powerup_sound = create_sound(880, 0.2)

class Player:
    def __init__(self):
        self.x = SCREEN_WIDTH // 2
        self.y = SCREEN_HEIGHT - 100
        self.width = 40
        self.height = 40
        self.speed = 5
        self.health = 100
        self.max_health = 100
        self.score = 0
        self.lives = 3
        self.shoot_cooldown = 0
        self.power_level = 1
        self.energy = 100
        self.max_energy = 100
        self.energy_regen = 0.5
        self.special_cooldown = 0
        self.invincible = 0
        self.invincible_timer = 0

    def draw(self, surface):
        # Основной корабль
        pygame.draw.polygon(surface, GREEN, [
            (self.x, self.y - 20),
            (self.x - 20, self.y + 20),
            (self.x + 20, self.y + 20)
        ])
        
        # Двигатели
        pygame.draw.rect(surface, YELLOW, (self.x - 8, self.y + 20, 5, 10))
        pygame.draw.rect(surface, YELLOW, (self.x + 3, self.y + 20, 5, 10))
        
        # Пушка
        pygame.draw.rect(surface, CYAN, (self.x - 5, self.y - 25, 10, 10))
        
        # Эффект невидимости
        if self.invincible > 0:
            pygame.draw.circle(surface, (100, 100, 255), (self.x, self.y), 25, 2)

    def move(self, keys):
        if keys[pygame.K_LEFT] and self.x > 20:
            self.x -= self.speed
        if keys[pygame.K_RIGHT] and self.x < SCREEN_WIDTH - 20:
            self.x += self.speed
        if keys[pygame.K_UP] and self.y > 20:
            self.y -= self.speed
        if keys[pygame.K_DOWN] and self.y < SCREEN_HEIGHT - 20:
            self.y += self.speed

    def shoot(self, bullets):
        if self.shoot_cooldown == 0:
            shoot_sound.play()
            if self.power_level == 1:
                bullets.append(Bullet(self.x, self.y - 20, 0, -10, BLUE))
            elif self.power_level == 2:
                bullets.extend([
                    Bullet(self.x - 10, self.y - 10, -3, -8, BLUE),
                    Bullet(self.x + 10, self.y - 10, 3, -8, BLUE)
                ])
            elif self.power_level >= 3:
                bullets.extend([
                    Bullet(self.x, self.y - 20, 0, -10, BLUE),
                    Bullet(self.x - 15, self.y - 10, -5, -7, BLUE),
                    Bullet(self.x + 15, self.y - 10, 5, -7, BLUE)
                ])
            self.shoot_cooldown = 10

    def special_attack(self, bullets):
        if self.special_cooldown == 0 and self.energy >= 50:
            self.energy -= 50
            self.special_cooldown = 300  # 5 секунд перезарядки
            explosion_sound.play()
            for i in range(12):
                angle = i * (360 / 12)
                rad = math.radians(angle)
                bullets.append(Bullet(self.x, self.y, math.cos(rad) * 5, math.sin(rad) * 5, YELLOW))

    def update(self, bullets):
        # Перезарядка энергии
        if self.energy < self.max_energy:
            self.energy += self.energy_regen
        
        # Обновление кулдаунов
        if self.shoot_cooldown > 0:
            self.shoot_cooldown -= 1
        if self.special_cooldown > 0:
            self.special_cooldown -= 1
        if self.invincible > 0:
            self.invincible -= 1

    def take_damage(self, amount):
        if self.invincible == 0:
            self.health -= amount
            self.invincible = 60  # 1 секунда невидимости
            if self.health <= 0:
                self.lives -= 1
                self.health = self.max_health
                self.invincible = 120  # 2 секунды невидимости после возрождения

class Enemy:
    def __init__(self, x, y, enemy_type):
        self.x = x
        self.y = y
        self.type = enemy_type
        self.speed = 2
        self.health = 1
        self.width = 30
        self.height = 30
        self.shoot_timer = 0
        self.direction = 1
        
        if enemy_type == 1:  # Обычный враг
            self.speed = 1
            self.color = RED
            self.health = 1
        elif enemy_type == 2:  # Быстрый враг
            self.speed = 3
            self.color = PURPLE
            self.health = 2
        elif enemy_type == 3:  # Сильный враг
            self.speed = 0.5
            self.color = CYAN
            self.health = 3

    def draw(self, surface):
        pygame.draw.rect(surface, self.color, (self.x - 15, self.y - 15, 30, 30))
        pygame.draw.circle(surface, WHITE, (self.x, self.y), 5)

    def move(self):
        self.y += self.speed

    def shoot(self, bullets):
        if self.type == 2:  # Быстрый враг стреляет чаще
            self.shoot_timer -= 1
            if self.shoot_timer <= 0:
                self.shoot_timer = 60
                bullets.append(Bullet(self.x, self.y + 15, 0, 5, RED))
        elif self.type == 3:  # Сильный враг стреляет по диагонали
            self.shoot_timer -= 1
            if self.shoot_timer <= 0:
                self.shoot_timer = 90
                for dx, dy in [(-3, 3), (0, 3), (3, 3)]:
                    bullets.append(Bullet(self.x, self.y + 15, dx, dy, RED))

    def update(self, bullets):
        self.move()
        if self.type != 1:  # Только необычные враги стреляют
            self.shoot(bullets)

class Bullet:
    def __init__(self, x, y, dx, dy, color):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.radius = 3
        self.color = color

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, (int(self.x), int(self.y)), self.radius)

    def move(self):
        self.x += self.dx
        self.y += self.dy

    def is_off_screen(self):
        return (self.x < 0 or self.x > SCREEN_WIDTH or 
                self.y < 0 or self.y > SCREEN_HEIGHT)

class PowerUp:
    def __init__(self, x, y, power_type):
        self.x = x
        self.y = y
        self.type = power_type  # 1-health, 2-power, 3-energy, 4-special
        self.width = 20
        self.height = 20
        self.speed = 2

    def draw(self, surface):
        if self.type == 1:
            color = GREEN
        elif self.type == 2:
            color = BLUE
        elif self.type == 3:
            color = YELLOW
        else:
            color = PURPLE
            
        pygame.draw.rect(surface, color, (self.x - 10, self.y - 10, 20, 20))
        pygame.draw.rect(surface, WHITE, (self.x - 10, self.y - 10, 20, 20), 1)

    def move(self):
        self.y += self.speed

class Particle:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
        self.size = random.randint(2, 5)
        self.speed_x = random.uniform(-3, 3)
        self.speed_y = random.uniform(-3, 3)
        self.life = random.randint(20, 40)

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, (int(self.x), int(self.y)), self.size)

    def update(self):
        self.x += self.speed_x
        self.y += self.speed_y
        self.life -= 1
        self.size = max(0, self.size - 0.1)
        return self.life <= 0

class Star:
    def __init__(self):
        self.x = random.randint(0, SCREEN_WIDTH)
        self.y = random.randint(0, SCREEN_HEIGHT)
        self.size = random.uniform(0.5, 2)
        self.speed = random.uniform(0.1, 0.5)
        self.brightness = random.randint(100, 255)

    def draw(self, surface):
        color = (self.brightness, self.brightness, self.brightness)
        pygame.draw.circle(surface, color, (int(self.x), int(self.y)), self.size)

    def update(self):
        self.y += self.speed
        if self.y > SCREEN_HEIGHT:
            self.y = 0
            self.x = random.randint(0, SCREEN_WIDTH)

class Game:
    def __init__(self):
        self.player = Player()
        self.enemies = []
        self.bullets = []
        self.enemy_bullets = []
        self.powerups = []
        self.particles = []
        self.stars = [Star() for _ in range(100)]
        self.game_state = "menu"  # menu, playing, game_over
        self.level = 1
        self.enemy_spawn_timer = 0
        self.high_score = 0
        self.load_high_score()

    def load_high_score(self):
        try:
            with open("highscore.txt", "r") as f:
                self.high_score = int(f.read())
        except:
            self.high_score = 0

    def save_high_score(self):
        if self.player.score > self.high_score:
            self.high_score = self.player.score
            with open("highscore.txt", "w") as f:
                f.write(str(self.high_score))

    def spawn_enemy(self):
        x = random.randint(50, SCREEN_WIDTH - 50)
        enemy_type = random.choices([1, 2, 3], weights=[70, 20, 10])[0]
        self.enemies.append(Enemy(x, -30, enemy_type))

    def spawn_powerup(self, x, y):
        power_type = random.choices([1, 2, 3, 4], weights=[30, 25, 25, 20])[0]
        self.powerups.append(PowerUp(x, y, power_type))

    def create_explosion(self, x, y, color):
        for _ in range(20):
            self.particles.append(Particle(x, y, color))
        explosion_sound.play()

    def check_collisions(self):
        # Пули игрока vs враги
        for bullet in self.bullets[:]:
            for enemy in self.enemies[:]:
                distance = math.sqrt((bullet.x - enemy.x)**2 + (bullet.y - enemy.y)**2)
                if distance < 20:
                    enemy.health -= 1
                    if enemy.health <= 0:
                        self.create_explosion(enemy.x, enemy.y, RED)
                        self.player.score += enemy.type * 10
                        if random.random() < 0.3:  # 30% шанс на бонус
                            self.spawn_powerup(enemy.x, enemy.y)
                        self.enemies.remove(enemy)
                    if bullet in self.bullets:
                        self.bullets.remove(bullet)
                    break

        # Вражеские пули vs игрок
        for bullet in self.enemy_bullets[:]:
            distance = math.sqrt((bullet.x - self.player.x)**2 + (bullet.y - self.player.y)**2)
            if distance < 20:
                self.player.take_damage(10)
                if bullet in self.enemy_bullets:
                    self.enemy_bullets.remove(bullet)
                if self.player.lives <= 0:
                    self.game_state = "game_over"
                    self.save_high_score()

        # Игрок vs враги
        for enemy in self.enemies[:]:
            distance = math.sqrt((enemy.x - self.player.x)**2 + (enemy.y - self.player.y)**2)
            if distance < 30:
                self.player.take_damage(20)
                self.create_explosion(enemy.x, enemy.y, RED)
                if enemy in self.enemies:
                    self.enemies.remove(enemy)
                if self.player.lives <= 0:
                    self.game_state = "game_over"
                    self.save_high_score()

        # Игрок vs бонусы
        for powerup in self.powerups[:]:
            distance = math.sqrt((powerup.x - self.player.x)**2 + (powerup.y - self.player.y)**2)
            if distance < 25:
                powerup_sound.play()
                if powerup.type == 1:  # Здоровье
                    self.player.health = min(self.player.max_health, self.player.health + 30)
                elif powerup.type == 2:  # Усиление оружия
                    self.player.power_level = min(3, self.player.power_level + 1)
                elif powerup.type == 3:  # Энергия
                    self.player.energy = min(self.player.max_energy, self.player.energy + 50)
                elif powerup.type == 4:  # Специальная атака
                    self.player.special_cooldown = max(0, self.player.special_cooldown - 100)
                
                self.powerups.remove(powerup)

    def update(self):
        if self.game_state != "playing":
            return

        # Обновление игрока
        self.player.update(self.bullets)

        # Обновление врагов
        for enemy in self.enemies[:]:
            enemy.update(self.enemy_bullets)
            if enemy.y > SCREEN_HEIGHT:
                self.enemies.remove(enemy)

        # Обновление пуль
        for bullet in self.bullets[:]:
            bullet.move()
            if bullet.is_off_screen():
                self.bullets.remove(bullet)

        for bullet in self.enemy_bullets[:]:
            bullet.move()
            if bullet.is_off_screen():
                self.enemy_bullets.remove(bullet)

        # Обновление бонусов
        for powerup in self.powerups[:]:
            powerup.move()
            if powerup.y > SCREEN_HEIGHT:
                self.powerups.remove(powerup)

        # Обновление частиц
        for particle in self.particles[:]:
            if particle.update():
                self.particles.remove(particle)

        # Обновление звезд
        for star in self.stars:
            star.update()

        # Спавн врагов
        self.enemy_spawn_timer += 1
        spawn_rate = max(30, 120 - self.level * 5)  # Сложнее с уровнем
        if self.enemy_spawn_timer >= spawn_rate:
            self.spawn_enemy()
            self.enemy_spawn_timer = 0

        # Повышение уровня
        if self.player.score >= self.level * 1000:
            self.level += 1

        self.check_collisions()

    def draw(self, surface):
        # Очистка экрана
        surface.fill(BLACK)

        if self.game_state == "menu":
            self.draw_menu(surface)
        elif self.game_state == "game_over":
            self.draw_game_over(surface)
        else:
            self.draw_game(surface)

    def draw_menu(self, surface):
        title = font_large.render("КОСМИЧЕСКАЯ СТРЕЛЯЛКА", True, WHITE)
        start = font_medium.render("Нажмите ПРОБЕЛ чтобы начать", True, GREEN)
        exit_text = font_medium.render("Нажмите ESC чтобы выйти", True, RED)

        surface.blit(title, (SCREEN_WIDTH//2 - title.get_width()//2, 200))
        surface.blit(start, (SCREEN_WIDTH//2 - start.get_width()//2, 300))
        surface.blit(exit_text, (SCREEN_WIDTH//2 - exit_text.get_width()//2, 350))

    def draw_game_over(self, surface):
        game_over = font_large.render("ИГРА ОКОНЧЕНА", True, RED)
        score = font_medium.render(f"Счёт: {self.player.score}", True, WHITE)
        high_score = font_medium.render(f"Рекорд: {self.high_score}", True, YELLOW)
        restart = font_medium.render("Нажмите R чтобы перезапустить", True, GREEN)
        exit_text = font_medium.render("Нажмите ESC чтобы выйти", True, RED)

        surface.blit(game_over, (SCREEN_WIDTH//2 - game_over.get_width()//2, 200))
        surface.blit(score, (SCREEN_WIDTH//2 - score.get_width()//2, 250))
        surface.blit(high_score, (SCREEN_WIDTH//2 - high_score.get_width()//2, 290))
        surface.blit(restart, (SCREEN_WIDTH//2 - restart.get_width()//2, 330))
        surface.blit(exit_text, (SCREEN_WIDTH//2 - exit_text.get_width()//2, 370))

    def draw_game(self, surface):
        # Рисуем звезды
        for star in self.stars:
            star.draw(surface)

        # Рисуем игрока
        self.player.draw(surface)

        # Рисуем врагов
        for enemy in self.enemies:
            enemy.draw(surface)

        # Рисуем пули
        for bullet in self.bullets:
            bullet.draw(surface)
        for bullet in self.enemy_bullets:
            bullet.draw(surface)

        # Рисуем бонусы
        for powerup in self.powerups:
            powerup.draw(surface)

        # Рисуем частицы
        for particle in self.particles:
            particle.draw(surface)

        # Рисуем UI
        health_bar = pygame.Rect(20, 20, 200, 20)
        pygame.draw.rect(surface, RED, health_bar)
        pygame.draw.rect(surface, GREEN, (20, 20, 200 * (self.player.health / self.player.max_health), 20))
        pygame.draw.rect(surface, WHITE, health_bar, 2)

        energy_bar = pygame.Rect(20, 50, 200, 15)
        pygame.draw.rect(surface, (50, 50, 50), energy_bar)
        pygame.draw.rect(surface, YELLOW, (20, 50, 200 * (self.player.energy / self.player.max_energy), 15))
        pygame.draw.rect(surface, WHITE, energy_bar, 1)

        score_text = font_small.render(f"Счёт: {self.player.score}", True, WHITE)
        level_text = font_small.render(f"Уровень: {self.level}", True, WHITE)
        lives_text = font_small.render(f"Жизни: {self.player.lives}", True, WHITE)
        special_text = font_small.render(f"Спец. атака: {'ГОТОВА' if self.player.special_cooldown == 0 and self.player.energy >= 50 else 'ЗАРЯДКА'}", True, WHITE)

        surface.blit(score_text, (20, 80))
        surface.blit(level_text, (20, 110))
        surface.blit(lives_text, (20, 140))
        surface.blit(special_text, (20, 170))

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return False
                if event.key == pygame.K_SPACE and self.game_state == "menu":
                    self.restart_game()
                if event.key == pygame.K_r and self.game_state == "game_over":
                    self.restart_game()
                if event.key == pygame.K_x and self.game_state == "playing":
                    self.player.special_attack(self.bullets)
        return True

    def restart_game(self):
        self.player = Player()
        self.enemies = []
        self.bullets = []
        self.enemy_bullets = []
        self.powerups = []
        self.particles = []
        self.stars = [Star() for _ in range(100)]
        self.game_state = "playing"
        self.level = 1
        self.enemy_spawn_timer = 0

def main():
    game = Game()
    running = True

    while running:
        running = game.handle_events()
        
        keys = pygame.key.get_pressed()
        if game.game_state == "playing":
            game.player.move(keys)
            if keys[pygame.K_z]:
                game.player.shoot(game.bullets)

        game.update()
        game.draw(screen)
        
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()