param([ValidateSet('diagnostic','normal')][string]$Mode='diagnostic',
      [string]$Report=(Join-Path $PSScriptRoot '..\MDK-ARM\cbt6_demo3\cbt6_demo3.htm'))
$ErrorActionPreference='Stop'
$html=Get-Content -LiteralPath $Report -Raw
$startup=Get-Content -LiteralPath (Join-Path $PSScriptRoot '..\MDK-ARM\startup_stm32f103xb.s') -Raw
$budgetMatch=[regex]::Match($startup,'Stack_Size\s+EQU\s+0x([0-9A-Fa-f]+)')
if(!$budgetMatch.Success){throw 'Missing stack allocation'}
$budget=[Convert]::ToInt32($budgetMatch.Groups[1].Value,16)
function Depth([string]$Name) {
    $pattern='(?s)<P><STRONG><a name="\[[^\]]+\]"></a>'+[regex]::Escape($Name)+'[^<]*</STRONG>.*?(?=<P><STRONG>|\z)'
    $block=[regex]::Match($html,$pattern)
    $value=[regex]::Match($block.Value,'Max Depth =\s*(\d+)')
    if(!$value.Success -and $Name -eq 'SysTick_Handler') {
        $leaf=[regex]::Match($html,'(?s)<P><STRONG><a name="\[[^\]]+\]"></a>HAL_IncTick</STRONG>.*?(?=<P><STRONG>|\z)')
        if($block.Value -match 'Stack size 0 bytes' -and $leaf.Value -match 'Stack size 0 bytes' -and $leaf.Value -notmatch '\[Calls\]') {return 0}
    }
    if(!$value.Success){throw "Missing call graph depth: $Name"}
    return [int]$value.Groups[1].Value
}
$main=Depth 'main'
# armlink cannot resolve the Parser Receiver function pointer. Conservatively
# add the handler's entire chain to the reported main depth (not just its frame).
$receiver=Depth '&lang;unnamed namespace 1&rang;::handleFrame'
$interrupts=@('USART1_IRQHandler','DMA1_Channel4_IRQHandler','DMA1_Channel5_IRQHandler')
if($Mode -eq 'normal') {$interrupts+=@('TIM4_IRQHandler','TIM1_UP_IRQHandler')}
$irq=0
foreach($name in $interrupts){$irq=[Math]::Max($irq,(Depth $name))}
$systick=Depth 'SysTick_Handler'
# USART/DMA/TIM IRQs use equal preemption priority, so do not nest each other.
# Allow two hardware exception frames, SysTick depth, plus 64B unknown-call margin.
$estimate=$main+$receiver+$irq+$systick+64+64
"STACK mode=$Mode budget=$budget main=$main receiver=$receiver irq=$irq systick=$systick allowance=128 estimate=$estimate"
if($estimate -gt $budget){throw 'Stack budget exceeded: do not flash this profile'}
'PASS: conservative static gate; not a substitute for on-target stack high-water measurement'
